// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <algorithm>
#include <deque>
#include <atomic>
#include <math.h>
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/killSafe.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/localization/localization.hpp"

// tracking thread
pros::Task* trackingTask = nullptr;

// global variables
lemlib::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr); // the sensors to be used for odometry
lemlib::Drivetrain drive(nullptr, nullptr, 0, 0, 0, 0); // the drivetrain to be used for odometry
lemlib::Pose odomPose(0, 0, 0); // the pose of the robot
lemlib::Pose odomSpeed(0, 0, 0); // the speed of the robot
lemlib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot
pros::Mutex odomStateMutex; // protects pose and speed state shared with localization
lemlib::OdomDelta odomDelta; // most recent local delta
lemlib::OdomTelemetry odomTelemetry; // raw inputs that produced the most recent odom delta
std::deque<lemlib::OdomDelta> odomDeltaHistory; // recent deltas for localization replay
std::deque<lemlib::OdomTelemetry> odomTelemetryHistory; // recent raw telemetry for offline tuning
pros::Mutex odomDeltaMutex; // protects odomDelta from torn reads
pros::Mutex odomUpdateMutex; // serializes setPose resets with live odom integration
// The three odom locks are taken blocking only on persistent tasks (update(),
// getOdomDeltasSince, getOdomTelemetryForSeq); every other path uses
// lemlib::withKillSafeLock, so a deleted competition task never owns one.
std::atomic<uint32_t> odomDeltaSeq {0};
uint32_t odomPoseSeq = 0;
uint32_t lastUpdateMs = 0;

float prevVertical = 0;
float prevVertical1 = 0;
float prevVertical2 = 0;
float prevHorizontal = 0;
float prevHorizontal1 = 0;
float prevHorizontal2 = 0;
float prevImu = 0;

namespace {
// Whether each per-wheel baseline above came from a real (not held) reading.
// Only odom update() and setPose's baseline apply touch these, both under
// odomUpdateMutex.
bool vertical1BaselineFresh = true;
bool vertical2BaselineFresh = true;
bool horizontal1BaselineFresh = true;
bool horizontal2BaselineFresh = true;
// The last IMU read was invalid, so prevImu predates an outage (same locking).
bool imuBaselineStale = false;

constexpr float kMinDt = 0.005f;
constexpr float kMinTurn = 1e-6f;
constexpr float kMinReasonableDelta = 2.0f;
constexpr float kFallbackMaxSpeedIps = 120.0f;
constexpr float kMaxSpeedMargin = 3.0f;
constexpr size_t kOdomDeltaHistorySize = 256;

bool isValidWheelReading(float reading) {
    // PROS_ERR sentinels arrive as NaN from TrackingWheel::getDistanceTraveled.
    // The magnitude bound is only a defensive backstop placed above rotation-
    // sensor saturation (~4e5 in for a 2.125" wheel at gearRatio 1), so a
    // legitimate long-session cumulative reading can never freeze odometry.
    return std::isfinite(reading) && std::fabs(reading) < 1e6f;
}

bool isValidImuReading(float reading) { return std::isfinite(reading) && std::fabs(reading) < 1e5f; }

float sanitizeReading(float reading, float previous) { return isValidWheelReading(reading) ? reading : previous; }

float readWheel(lemlib::TrackingWheel* wheel, float previous, bool& valid) {
    const float reading = wheel->getDistanceTraveled();
    valid = isValidWheelReading(reading);
    return valid ? reading : previous;
}

float clampMagnitude(float value, float magnitude) { return std::clamp(value, -magnitude, magnitude); }

float wrapHeadingDelta(float delta) { return std::remainder(delta, 2.0f * static_cast<float>(M_PI)); }

float computeDt(uint32_t nowMs) {
    if (lastUpdateMs == 0) {
        lastUpdateMs = nowMs;
        return 0.01f;
    }

    const float rawDt = static_cast<float>(nowMs - lastUpdateMs) / 1000.0f;
    lastUpdateMs = nowMs;
    return std::max(rawDt, kMinDt);
}

float computeMaxDelta(float dt) {
    float maxSpeedIps = kFallbackMaxSpeedIps;
    if (drive.wheelDiameter > 0 && drive.rpm > 0) {
        const float drivetrainSpeedIps = drive.wheelDiameter * static_cast<float>(M_PI) * drive.rpm / 60.0f;
        maxSpeedIps = std::max(maxSpeedIps, drivetrainSpeedIps * kMaxSpeedMargin);
    }
    return std::max(kMinReasonableDelta, maxSpeedIps * dt);
}

float computeMaxHeadingDelta(float dt) {
    float maxTurnRate = 12.0f; // rad/s fallback
    if (drive.trackWidth > 0 && drive.wheelDiameter > 0 && drive.rpm > 0) {
        const float drivetrainSpeedIps = drive.wheelDiameter * static_cast<float>(M_PI) * drive.rpm / 60.0f;
        maxTurnRate = std::max(maxTurnRate, (2.0f * drivetrainSpeedIps / drive.trackWidth) * kMaxSpeedMargin);
    }
    return std::max(0.05f, maxTurnRate * dt);
}

// Differential heading from a wheel pair. A member whose delta is not a real
// one-tick sample (held reading, catch-up after a dropout, or a rejected jump
// zeroed by the caller) would turn its partner's motion into fabricated
// rotation, so the pair is refused instead; so is an implied rotation beyond
// the per-tick bound (reject, don't clamp). On refusal `heading` is untouched
// and the caller falls through to the next heading source.
bool tryHeadingFromPair(float deltaA, bool usableA, float deltaB, bool usableB, lemlib::TrackingWheel* sensorA,
                        lemlib::TrackingWheel* sensorB, float maxHeadingDelta, float& heading) {
    if (sensorA == nullptr || sensorB == nullptr || !usableA || !usableB) return false;
    const float offsetDelta = sensorA->getOffset() - sensorB->getOffset();
    if (std::fabs(offsetDelta) < kMinTurn) return false;
    const float deltaHeading = -(deltaA - deltaB) / offsetDelta;
    if (!std::isfinite(deltaHeading) || std::fabs(deltaHeading) > maxHeadingDelta) return false;
    heading += deltaHeading;
    return true;
}

lemlib::TrackingWheel* selectVerticalWheel() {
    if (odomSensors.vertical1 != nullptr && !odomSensors.vertical1->getType()) return odomSensors.vertical1;
    if (odomSensors.vertical2 != nullptr && !odomSensors.vertical2->getType()) return odomSensors.vertical2;
    if (odomSensors.vertical1 != nullptr) return odomSensors.vertical1;
    return odomSensors.vertical2;
}

lemlib::TrackingWheel* selectHorizontalWheel() {
    if (odomSensors.horizontal1 != nullptr) return odomSensors.horizontal1;
    return odomSensors.horizontal2;
}

struct RawSensorBaselines {
    float vertical1 = 0;
    float vertical2 = 0;
    float horizontal1 = 0;
    float horizontal2 = 0;
    float imu = 0;
    float vertical = 0;
    float horizontal = 0;
};

// Device reads only; touches no shared odom state, so setPose can run it before
// taking the odom locks.
RawSensorBaselines readSensorBaselines() {
    RawSensorBaselines raw {};
    if (odomSensors.vertical1 != nullptr) raw.vertical1 = odomSensors.vertical1->getDistanceTraveled();
    if (odomSensors.vertical2 != nullptr) raw.vertical2 = odomSensors.vertical2->getDistanceTraveled();
    if (odomSensors.horizontal1 != nullptr) raw.horizontal1 = odomSensors.horizontal1->getDistanceTraveled();
    if (odomSensors.horizontal2 != nullptr) raw.horizontal2 = odomSensors.horizontal2->getDistanceTraveled();
    if (odomSensors.imu != nullptr) raw.imu = lemlib::degToRad(odomSensors.imu->get_rotation());
    if (lemlib::TrackingWheel* wheel = selectVerticalWheel(); wheel != nullptr)
        raw.vertical = wheel->getDistanceTraveled();
    if (lemlib::TrackingWheel* wheel = selectHorizontalWheel(); wheel != nullptr)
        raw.horizontal = wheel->getDistanceTraveled();
    return raw;
}

// The caller holds odomUpdateMutex, inside setPose's kill-safe region: pure math
// and TrackingWheel::getType only.
void applySensorBaselines(const RawSensorBaselines& raw) {
    prevVertical1 = (odomSensors.vertical1 != nullptr) ? sanitizeReading(raw.vertical1, prevVertical1) : 0.0f;
    prevVertical2 = (odomSensors.vertical2 != nullptr) ? sanitizeReading(raw.vertical2, prevVertical2) : 0.0f;
    prevHorizontal1 = (odomSensors.horizontal1 != nullptr) ? sanitizeReading(raw.horizontal1, prevHorizontal1) : 0.0f;
    prevHorizontal2 = (odomSensors.horizontal2 != nullptr) ? sanitizeReading(raw.horizontal2, prevHorizontal2) : 0.0f;
    vertical1BaselineFresh = isValidWheelReading(raw.vertical1);
    vertical2BaselineFresh = isValidWheelReading(raw.vertical2);
    horizontal1BaselineFresh = isValidWheelReading(raw.horizontal1);
    horizontal2BaselineFresh = isValidWheelReading(raw.horizontal2);
    if (odomSensors.imu == nullptr) {
        prevImu = 0.0f;
        imuBaselineStale = false;
    } else if (isValidImuReading(raw.imu)) {
        prevImu = raw.imu;
        imuBaselineStale = false;
    } else {
        // Keep the old baseline but never difference the next valid read against it.
        imuBaselineStale = true;
    }

    prevVertical = (selectVerticalWheel() != nullptr) ? sanitizeReading(raw.vertical, prevVertical) : 0.0f;
    prevHorizontal = (selectHorizontalWheel() != nullptr) ? sanitizeReading(raw.horizontal, prevHorizontal) : 0.0f;
    lastUpdateMs = pros::millis();
}
} // namespace

void lemlib::setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain) {
    odomSensors = sensors;
    drive = drivetrain;
}

lemlib::Pose lemlib::getPose(bool radians) {
    Pose pose(0, 0, 0);
    withKillSafeLock([&]() noexcept { pose = odomPose; }, odomStateMutex);
    if (radians) return pose;
    else return lemlib::Pose(pose.x, pose.y, radToDeg(pose.theta));
}

namespace {
void setPoseImpl(lemlib::Pose pose, bool radians, bool syncLocalization, bool resetOdomDelta) {
    const lemlib::Pose poseRad = radians ? pose : lemlib::Pose(pose.x, pose.y, lemlib::degToRad(pose.theta));
    uint32_t newSeq = 0;
    // Device reads and allocator work stay outside the kill-safe region; the reset
    // is never half-applied and a competition task can no longer orphan an odom lock.
    std::deque<lemlib::OdomDelta> retiredDeltaHistory;
    std::deque<lemlib::OdomTelemetry> retiredTelemetryHistory;
    RawSensorBaselines rawBaselines {};
    if (resetOdomDelta) rawBaselines = readSensorBaselines();

    if (resetOdomDelta) {
        lemlib::withKillSafeLock(
            [&]() noexcept {
                newSeq = odomDeltaSeq.fetch_add(1) + 1;
                odomDelta.localX = 0;
                odomDelta.localY = 0;
                odomDelta.deltaTheta = 0;
                odomDelta.dt = 0.01f;
                odomDelta.seq = newSeq;
                odomTelemetry = {};
                odomTelemetry.seq = newSeq;
                odomDeltaHistory.swap(retiredDeltaHistory);
                odomTelemetryHistory.swap(retiredTelemetryHistory);
                // Invalidate corrections staged against the old frame in the SAME
                // critical section that publishes the new pose/seq: any snapshot that
                // observes the new frame then also observes the new epoch (see
                // applyStagedBoundaryReanchor).
                lemlib::localization::invalidateCorrectionFrame();
                odomPose = poseRad;
                odomSpeed = lemlib::Pose(0, 0, 0);
                odomLocalSpeed = lemlib::Pose(0, 0, 0);
                odomPoseSeq = newSeq;
                applySensorBaselines(rawBaselines);
            },
            odomUpdateMutex, odomDeltaMutex, odomStateMutex);
    } else {
        lemlib::withKillSafeLock([&]() noexcept { odomPose = poseRad; }, odomStateMutex);
    }

    if (syncLocalization) {
        if (resetOdomDelta) lemlib::localization::syncPose(poseRad, newSeq);
        else lemlib::localization::syncPose(poseRad);
    }
}
} // namespace

void lemlib::setPose(lemlib::Pose pose, bool radians) { setPoseImpl(pose, radians, true, true); }

void lemlib::detail::setPoseSilent(lemlib::Pose pose, bool radians) { setPoseImpl(pose, radians, false, false); }

bool lemlib::detail::setPoseSilentIfSeq(lemlib::Pose pose, uint32_t expectedSeq, bool radians, bool (*abortIf)()) {
    const lemlib::Pose poseRad = radians ? pose : lemlib::Pose(pose.x, pose.y, lemlib::degToRad(pose.theta));
    // Same lock order as lemlib::update() (odomUpdateMutex -> odomStateMutex) so
    // no odom integration can interleave between the seq check and the write.
    bool seqMatches = false;
    // The abort predicate is evaluated under the odom locks: callers use it to
    // re-check a condition (e.g. motion-correction suppression) that another
    // task may have changed between building the pose and committing it. A
    // motion's first getPose also takes odomStateMutex, so a write that passes
    // this check is ordered before any pose read of the motion that suppressed.
    // abortIf runs scheduler-suspended: atomic load only.
    withKillSafeLock(
        [&]() noexcept {
            const bool aborted = (abortIf != nullptr) && abortIf();
            seqMatches = !aborted && (odomPoseSeq == expectedSeq);
            if (seqMatches) odomPose = poseRad;
        },
        odomUpdateMutex, odomStateMutex);
    return seqMatches;
}

lemlib::Pose lemlib::getSpeed(bool radians) {
    Pose speed(0, 0, 0);
    withKillSafeLock([&]() noexcept { speed = odomSpeed; }, odomStateMutex);
    if (radians) return speed;
    else return lemlib::Pose(speed.x, speed.y, radToDeg(speed.theta));
}

lemlib::Pose lemlib::getLocalSpeed(bool radians) {
    Pose localSpeed(0, 0, 0);
    withKillSafeLock([&]() noexcept { localSpeed = odomLocalSpeed; }, odomStateMutex);
    if (radians) return localSpeed;
    else return lemlib::Pose(localSpeed.x, localSpeed.y, radToDeg(localSpeed.theta));
}

lemlib::OdomDelta lemlib::getOdomDelta() {
    OdomDelta copy;
    withKillSafeLock([&]() noexcept { copy = odomDelta; }, odomDeltaMutex);
    return copy;
}

// Blocking take, allocates under the lock: persistent tasks only.
std::vector<lemlib::OdomDelta> lemlib::getOdomDeltasSince(uint32_t seq) {
    odomDeltaMutex.take();
    std::vector<OdomDelta> deltas;
    deltas.reserve(odomDeltaHistory.size());
    for (const OdomDelta& delta : odomDeltaHistory) {
        if (delta.seq > seq) deltas.push_back(delta);
    }
    odomDeltaMutex.give();
    return deltas;
}

lemlib::OdomSnapshot lemlib::getOdomSnapshot() {
    OdomSnapshot snapshot;
    withKillSafeLock([&]() noexcept { snapshot = {.pose = odomPose, .seq = odomPoseSeq}; }, odomStateMutex);
    return snapshot;
}

lemlib::OdomTelemetry lemlib::getOdomTelemetry() {
    OdomTelemetry copy;
    withKillSafeLock([&]() noexcept { copy = odomTelemetry; }, odomDeltaMutex);
    return copy;
}

// Blocking take, scans the history under the lock: persistent tasks only.
lemlib::OdomTelemetry lemlib::getOdomTelemetryForSeq(uint32_t seq) {
    odomDeltaMutex.take();
    OdomTelemetry copy {};
    for (const OdomTelemetry& telemetry : odomTelemetryHistory) {
        if (telemetry.seq == seq) {
            copy = telemetry;
            break;
        }
    }
    odomDeltaMutex.give();
    return copy;
}

lemlib::Pose lemlib::estimatePose(float time, bool radians) {
    // get current position and speed
    const Pose curPose = getPose(true);
    const Pose localSpeed = getLocalSpeed(true);
    const float deltaLocalX = localSpeed.x * time;
    const float deltaLocalY = localSpeed.y * time;
    const float deltaTheta = localSpeed.theta * time;

    // calculate the future pose
    const float avgHeading = curPose.theta + deltaTheta / 2;
    Pose futurePose = curPose;
    futurePose.x += deltaLocalY * sin(avgHeading);
    futurePose.y += deltaLocalY * cos(avgHeading);
    futurePose.x += deltaLocalX * -cos(avgHeading);
    futurePose.y += deltaLocalX * sin(avgHeading);
    futurePose.theta += deltaTheta;
    if (!radians) futurePose.theta = radToDeg(futurePose.theta);

    return futurePose;
}

void lemlib::update() {
    odomUpdateMutex.take();
    const uint32_t nowMs = pros::millis();
    const float dt = computeDt(nowMs);
    const float maxDeltaPerUpdate = computeMaxDelta(dt);
    const float maxHeadingDelta = computeMaxHeadingDelta(dt);
    // get the current sensor values
    float vertical1Raw = 0;
    float vertical2Raw = 0;
    float horizontal1Raw = 0;
    float horizontal2Raw = 0;
    float imuRaw = 0;
    bool vertical1Valid = false;
    bool vertical2Valid = false;
    bool horizontal1Valid = false;
    bool horizontal2Valid = false;
    bool imuReadingValid = false;
    if (odomSensors.vertical1 != nullptr)
        vertical1Raw = readWheel(odomSensors.vertical1, prevVertical1, vertical1Valid);
    if (odomSensors.vertical2 != nullptr)
        vertical2Raw = readWheel(odomSensors.vertical2, prevVertical2, vertical2Valid);
    if (odomSensors.horizontal1 != nullptr)
        horizontal1Raw = readWheel(odomSensors.horizontal1, prevHorizontal1, horizontal1Valid);
    if (odomSensors.horizontal2 != nullptr)
        horizontal2Raw = readWheel(odomSensors.horizontal2, prevHorizontal2, horizontal2Valid);
    if (odomSensors.imu != nullptr) {
        const float candidate = degToRad(odomSensors.imu->get_rotation());
        imuReadingValid = isValidImuReading(candidate);
        imuRaw = imuReadingValid ? candidate : prevImu;
    }

    // calculate the change in sensor values
    float deltaVertical1 = vertical1Raw - prevVertical1;
    float deltaVertical2 = vertical2Raw - prevVertical2;
    float deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
    float deltaHorizontal2 = horizontal2Raw - prevHorizontal2;
    float deltaImu = imuRaw - prevImu;
    // The first valid IMU read after any invalid one only rebaselines. An IMU
    // reboot (cable/ESD) restarts rotation near 0 after recalibrating, and the
    // outage rotation was already carried by the drivetrain pair (or lost to a
    // hold), so differencing against the pre-outage baseline would inject a
    // phantom or double-counted heading step.
    const bool imuDeltaRejected = odomSensors.imu != nullptr &&
                                  (!imuReadingValid || imuBaselineStale || std::fabs(deltaImu) > maxHeadingDelta);

    // A wheel feeds a differential heading pair only with a real one-tick delta:
    // a held reading, the catch-up delta after a dropout, or a rejected jump
    // (zeroed below) would turn its partner's motion into fabricated rotation.
    const bool vertical1PairOk =
        vertical1Valid && vertical1BaselineFresh && std::fabs(deltaVertical1) <= maxDeltaPerUpdate;
    const bool vertical2PairOk =
        vertical2Valid && vertical2BaselineFresh && std::fabs(deltaVertical2) <= maxDeltaPerUpdate;
    const bool horizontal1PairOk =
        horizontal1Valid && horizontal1BaselineFresh && std::fabs(deltaHorizontal1) <= maxDeltaPerUpdate;
    const bool horizontal2PairOk =
        horizontal2Valid && horizontal2BaselineFresh && std::fabs(deltaHorizontal2) <= maxDeltaPerUpdate;

    // Encoder sanity check: reject impossibly large deltas outright (e.g. from a
    // sensor that reset its position or a glitch below the sanitize gate).
    // Clamping instead of rejecting would inject the clamp magnitude into the
    // pose as a fabricated step. Rebaselining after rejection lets a sensor that
    // reset its accumulated position resume on the next tick.
    if (std::fabs(deltaVertical1) > maxDeltaPerUpdate) deltaVertical1 = 0;
    if (std::fabs(deltaVertical2) > maxDeltaPerUpdate) deltaVertical2 = 0;
    if (std::fabs(deltaHorizontal1) > maxDeltaPerUpdate) deltaHorizontal1 = 0;
    if (std::fabs(deltaHorizontal2) > maxDeltaPerUpdate) deltaHorizontal2 = 0;
    // Rebaseline even after a rejected finite jump. Otherwise a sensor that
    // reset to zero stays frozen until it re-accumulates its entire old reading.
    prevVertical1 = vertical1Raw;
    prevVertical2 = vertical2Raw;
    prevHorizontal1 = horizontal1Raw;
    prevHorizontal2 = horizontal2Raw;
    vertical1BaselineFresh = vertical1Valid;
    vertical2BaselineFresh = vertical2Valid;
    horizontal1BaselineFresh = horizontal1Valid;
    horizontal2BaselineFresh = horizontal2Valid;
    if (imuReadingValid) prevImu = imuRaw;
    if (odomSensors.imu != nullptr) imuBaselineStale = !imuReadingValid;
    if (imuDeltaRejected) deltaImu = 0.0f;

    const uint32_t newSeq = odomDeltaSeq.fetch_add(1) + 1;

    odomStateMutex.take();

    // calculate the heading of the robot
    // Priority:
    // 1. Horizontal tracking wheel pair
    // 2. Vertical tracking wheel pair (both non-powered)
    // 3. Inertial Sensor
    // 4. Drivetrain (vertical pair incl. powered wheels) whenever the IMU gives
    //    no usable delta this tick: nulled after failed calibration, invalid
    //    (disconnect/recalibration), rebaselining after an outage, or an
    //    impossible jump
    // 5. Hold last heading (no usable pair either)
    const float headingBefore = odomPose.theta;
    float heading = odomPose.theta;
    const bool usedHorizontalHeading =
        tryHeadingFromPair(deltaHorizontal1, horizontal1PairOk, deltaHorizontal2, horizontal2PairOk,
                           odomSensors.horizontal1, odomSensors.horizontal2, maxHeadingDelta, heading);
    const bool canUseVerticalHeading = odomSensors.vertical1 != nullptr && odomSensors.vertical2 != nullptr &&
                                       !odomSensors.vertical1->getType() && !odomSensors.vertical2->getType();
    bool usedVerticalHeading = false;
    bool usedImuHeading = false;
    bool usedHeadingFallback = false;
    if (!usedHorizontalHeading) {
        usedVerticalHeading =
            canUseVerticalHeading && tryHeadingFromPair(deltaVertical1, vertical1PairOk, deltaVertical2,
                                                        vertical2PairOk, odomSensors.vertical1, odomSensors.vertical2,
                                                        maxHeadingDelta, heading);
        if (!usedVerticalHeading) {
            if (odomSensors.imu != nullptr && !imuDeltaRejected) {
                heading += deltaImu;
                usedImuHeading = true;
            } else if (tryHeadingFromPair(deltaVertical1, vertical1PairOk, deltaVertical2, vertical2PairOk,
                                          odomSensors.vertical1, odomSensors.vertical2, maxHeadingDelta, heading)) {
                // Drivetrain fallback: the IMU is nulled or gave no usable delta
                // this tick, so the slip-prone powered vertical pair (vertical2 is
                // substituted from drive motors at calibrate()) is the only heading
                // source left. Freezing heading here instead would keep integrating
                // translation against a dead heading and manufacture ~|offset|
                // inches of phantom motion per radian of real rotation, for as long
                // as an IMU outage lasts.
                usedVerticalHeading = true;
            } else {
                heading = odomPose.theta;
                usedHeadingFallback = true;
            }
        }
    }
    float deltaHeading = clampMagnitude(heading - odomPose.theta, maxHeadingDelta);
    heading = odomPose.theta + deltaHeading;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    // choose tracking wheels to use
    // Prioritize non-powered tracking wheels
    lemlib::TrackingWheel* verticalWheel = nullptr;
    lemlib::TrackingWheel* horizontalWheel = nullptr;
    if (odomSensors.vertical1 != nullptr && !odomSensors.vertical1->getType()) verticalWheel = odomSensors.vertical1;
    else if (odomSensors.vertical2 != nullptr && !odomSensors.vertical2->getType())
        verticalWheel = odomSensors.vertical2;
    else if (odomSensors.vertical1 != nullptr) verticalWheel = odomSensors.vertical1;
    else verticalWheel = odomSensors.vertical2;
    if (odomSensors.horizontal1 != nullptr) horizontalWheel = odomSensors.horizontal1;
    else if (odomSensors.horizontal2 != nullptr) horizontalWheel = odomSensors.horizontal2;
    float rawVertical = 0;
    float rawHorizontal = 0;
    if (verticalWheel != nullptr) rawVertical = sanitizeReading(verticalWheel->getDistanceTraveled(), prevVertical);
    if (horizontalWheel != nullptr)
        rawHorizontal = sanitizeReading(horizontalWheel->getDistanceTraveled(), prevHorizontal);
    float horizontalOffset = 0;
    float verticalOffset = 0;
    if (verticalWheel != nullptr) verticalOffset = verticalWheel->getOffset();
    if (horizontalWheel != nullptr) horizontalOffset = horizontalWheel->getOffset();

    // calculate change in x and y
    float deltaX = 0;
    float deltaY = 0;
    if (verticalWheel != nullptr) deltaY = rawVertical - prevVertical;
    if (horizontalWheel != nullptr) deltaX = rawHorizontal - prevHorizontal;
    // Same reject-don't-clamp policy as the per-wheel deltas above: a fabricated
    // clamp-magnitude step would poison the pose; rebaseline so a reset sensor
    // resumes from its new accumulated-position origin.
    if (std::fabs(deltaY) > maxDeltaPerUpdate) deltaY = 0;
    if (std::fabs(deltaX) > maxDeltaPerUpdate) deltaX = 0;
    prevVertical = rawVertical;
    prevHorizontal = rawHorizontal;

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (std::fabs(deltaHeading) < kMinTurn) { // prevent divide by ~0
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    // save previous pose
    lemlib::Pose prevPose = odomPose;

    // calculate global x and y
    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta = heading;

    // calculate speed
    odomSpeed.x = ema((odomPose.x - prevPose.x) / dt, odomSpeed.x, 0.95);
    odomSpeed.y = ema((odomPose.y - prevPose.y) / dt, odomSpeed.y, 0.95);
    odomSpeed.theta = ema(wrapHeadingDelta(odomPose.theta - prevPose.theta) / dt, odomSpeed.theta, 0.95);

    // calculate local speed
    odomLocalSpeed.x = ema(localX / dt, odomLocalSpeed.x, 0.95);
    odomLocalSpeed.y = ema(localY / dt, odomLocalSpeed.y, 0.95);
    odomLocalSpeed.theta = ema(deltaHeading / dt, odomLocalSpeed.theta, 0.95);
    odomPoseSeq = newSeq;
    odomStateMutex.give();

    // update odometry delta
    const OdomDelta newDelta {.localX = localX, .localY = localY, .deltaTheta = deltaHeading, .dt = dt, .seq = newSeq};
    const OdomTelemetry newTelemetry {
        .dt = dt,
        .headingBefore = headingBefore,
        .headingAfter = heading,
        .deltaHeading = deltaHeading,
        .vertical1Raw = vertical1Raw,
        .vertical2Raw = vertical2Raw,
        .horizontal1Raw = horizontal1Raw,
        .horizontal2Raw = horizontal2Raw,
        .imuRaw = imuRaw,
        .deltaVertical1 = deltaVertical1,
        .deltaVertical2 = deltaVertical2,
        .deltaHorizontal1 = deltaHorizontal1,
        .deltaHorizontal2 = deltaHorizontal2,
        .deltaImu = deltaImu,
        .selectedVerticalRaw = rawVertical,
        .selectedHorizontalRaw = rawHorizontal,
        .selectedDeltaVertical = deltaY,
        .selectedDeltaHorizontal = deltaX,
        .selectedVerticalOffset = verticalOffset,
        .selectedHorizontalOffset = horizontalOffset,
        .usedHorizontalHeadingPair = usedHorizontalHeading,
        .usedVerticalHeadingPair = usedVerticalHeading,
        .usedImuHeading = usedImuHeading,
        .usedHeadingFallback = usedHeadingFallback,
        .seq = newSeq,
    };
    odomDeltaMutex.take();
    odomDelta = newDelta;
    odomTelemetry = newTelemetry;
    odomDeltaHistory.push_back(newDelta);
    odomTelemetryHistory.push_back(newTelemetry);
    while (odomDeltaHistory.size() > kOdomDeltaHistorySize) { odomDeltaHistory.pop_front(); }
    while (odomTelemetryHistory.size() > kOdomDeltaHistorySize) { odomTelemetryHistory.pop_front(); }
    odomDeltaMutex.give();
    odomUpdateMutex.give();
}

void lemlib::init() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
    // start localization if configured
    lemlib::localization::start();
}
