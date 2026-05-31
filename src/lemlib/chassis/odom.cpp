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
constexpr float kMinDt = 0.005f;
constexpr float kMinTurn = 1e-6f;
constexpr float kMinReasonableDelta = 2.0f;
constexpr float kFallbackMaxSpeedIps = 120.0f;
constexpr float kMaxSpeedMargin = 3.0f;
constexpr size_t kOdomDeltaHistorySize = 256;

float sanitizeReading(float reading, float previous) {
    if (std::isfinite(reading) && std::fabs(reading) < 1e6f) return reading;
    return previous;
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

bool tryHeadingFromPair(float deltaA, float deltaB, lemlib::TrackingWheel* sensorA, lemlib::TrackingWheel* sensorB,
                        float& heading) {
    if (sensorA == nullptr || sensorB == nullptr) return false;
    const float offsetDelta = sensorA->getOffset() - sensorB->getOffset();
    if (std::fabs(offsetDelta) < kMinTurn) return false;
    heading -= (deltaA - deltaB) / offsetDelta;
    return std::isfinite(heading);
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

void captureSensorBaselines() {
    prevVertical1 = (odomSensors.vertical1 != nullptr)
                        ? sanitizeReading(odomSensors.vertical1->getDistanceTraveled(), prevVertical1)
                        : 0.0f;
    prevVertical2 = (odomSensors.vertical2 != nullptr)
                        ? sanitizeReading(odomSensors.vertical2->getDistanceTraveled(), prevVertical2)
                        : 0.0f;
    prevHorizontal1 = (odomSensors.horizontal1 != nullptr)
                          ? sanitizeReading(odomSensors.horizontal1->getDistanceTraveled(), prevHorizontal1)
                          : 0.0f;
    prevHorizontal2 = (odomSensors.horizontal2 != nullptr)
                          ? sanitizeReading(odomSensors.horizontal2->getDistanceTraveled(), prevHorizontal2)
                          : 0.0f;
    prevImu = (odomSensors.imu != nullptr) ? sanitizeReading(lemlib::degToRad(odomSensors.imu->get_rotation()), prevImu)
                                           : 0.0f;

    lemlib::TrackingWheel* verticalWheel = selectVerticalWheel();
    lemlib::TrackingWheel* horizontalWheel = selectHorizontalWheel();
    prevVertical =
        (verticalWheel != nullptr) ? sanitizeReading(verticalWheel->getDistanceTraveled(), prevVertical) : 0.0f;
    prevHorizontal =
        (horizontalWheel != nullptr) ? sanitizeReading(horizontalWheel->getDistanceTraveled(), prevHorizontal) : 0.0f;
    lastUpdateMs = pros::millis();
}
} // namespace

void lemlib::setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain) {
    odomSensors = sensors;
    drive = drivetrain;
}

lemlib::Pose lemlib::getPose(bool radians) {
    odomStateMutex.take();
    const Pose pose = odomPose;
    odomStateMutex.give();
    if (radians) return pose;
    else return lemlib::Pose(pose.x, pose.y, radToDeg(pose.theta));
}

namespace {
void setPoseImpl(lemlib::Pose pose, bool radians, bool syncLocalization, bool resetOdomDelta) {
    const lemlib::Pose poseRad = radians ? pose : lemlib::Pose(pose.x, pose.y, lemlib::degToRad(pose.theta));
    uint32_t newSeq = odomPoseSeq;

    if (resetOdomDelta) odomUpdateMutex.take();

    if (resetOdomDelta) {
        odomDeltaMutex.take();
        newSeq = odomDeltaSeq.fetch_add(1) + 1;
        odomDelta.localX = 0;
        odomDelta.localY = 0;
        odomDelta.deltaTheta = 0;
        odomDelta.dt = 0.01f;
        odomDelta.seq = newSeq;
        odomTelemetry = {};
        odomTelemetry.seq = newSeq;
        odomDeltaHistory.clear();
        odomTelemetryHistory.clear();
        odomDeltaMutex.give();
    }

    odomStateMutex.take();
    odomPose = poseRad;
    if (resetOdomDelta) {
        odomSpeed = lemlib::Pose(0, 0, 0);
        odomLocalSpeed = lemlib::Pose(0, 0, 0);
        odomPoseSeq = newSeq;
    }
    odomStateMutex.give();

    if (resetOdomDelta) {
        captureSensorBaselines();
        odomUpdateMutex.give();
    }

    if (syncLocalization) {
        if (resetOdomDelta) lemlib::localization::syncPose(poseRad, newSeq);
        else lemlib::localization::syncPose(poseRad);
    }
}
} // namespace

void lemlib::setPose(lemlib::Pose pose, bool radians) { setPoseImpl(pose, radians, true, true); }

void lemlib::detail::setPoseSilent(lemlib::Pose pose, bool radians) { setPoseImpl(pose, radians, false, false); }

bool lemlib::detail::setPoseSilentIfSeq(lemlib::Pose pose, uint32_t expectedSeq, bool radians) {
    const lemlib::Pose poseRad = radians ? pose : lemlib::Pose(pose.x, pose.y, lemlib::degToRad(pose.theta));
    // Same lock order as lemlib::update() (odomUpdateMutex -> odomStateMutex) so
    // no odom integration can interleave between the seq check and the write.
    odomUpdateMutex.take();
    odomStateMutex.take();
    const bool seqMatches = (odomPoseSeq == expectedSeq);
    if (seqMatches) odomPose = poseRad;
    odomStateMutex.give();
    odomUpdateMutex.give();
    return seqMatches;
}

lemlib::Pose lemlib::getSpeed(bool radians) {
    odomStateMutex.take();
    const Pose speed = odomSpeed;
    odomStateMutex.give();
    if (radians) return speed;
    else return lemlib::Pose(speed.x, speed.y, radToDeg(speed.theta));
}

lemlib::Pose lemlib::getLocalSpeed(bool radians) {
    odomStateMutex.take();
    const Pose localSpeed = odomLocalSpeed;
    odomStateMutex.give();
    if (radians) return localSpeed;
    else return lemlib::Pose(localSpeed.x, localSpeed.y, radToDeg(localSpeed.theta));
}

lemlib::OdomDelta lemlib::getOdomDelta() {
    odomDeltaMutex.take();
    const OdomDelta copy = odomDelta;
    odomDeltaMutex.give();
    return copy;
}

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
    odomStateMutex.take();
    const OdomSnapshot snapshot {.pose = odomPose, .seq = odomPoseSeq};
    odomStateMutex.give();
    return snapshot;
}

lemlib::OdomTelemetry lemlib::getOdomTelemetry() {
    odomDeltaMutex.take();
    const OdomTelemetry copy = odomTelemetry;
    odomDeltaMutex.give();
    return copy;
}

lemlib::OdomTelemetry lemlib::getOdomTelemetryForSeq(uint32_t seq) {
    odomDeltaMutex.take();
    OdomTelemetry copy = odomTelemetry;
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
    // TODO: add particle filter
    // get the current sensor values
    float vertical1Raw = 0;
    float vertical2Raw = 0;
    float horizontal1Raw = 0;
    float horizontal2Raw = 0;
    float imuRaw = 0;
    if (odomSensors.vertical1 != nullptr)
        vertical1Raw = sanitizeReading(odomSensors.vertical1->getDistanceTraveled(), prevVertical1);
    if (odomSensors.vertical2 != nullptr)
        vertical2Raw = sanitizeReading(odomSensors.vertical2->getDistanceTraveled(), prevVertical2);
    if (odomSensors.horizontal1 != nullptr)
        horizontal1Raw = sanitizeReading(odomSensors.horizontal1->getDistanceTraveled(), prevHorizontal1);
    if (odomSensors.horizontal2 != nullptr)
        horizontal2Raw = sanitizeReading(odomSensors.horizontal2->getDistanceTraveled(), prevHorizontal2);
    if (odomSensors.imu != nullptr) imuRaw = sanitizeReading(degToRad(odomSensors.imu->get_rotation()), prevImu);

    // calculate the change in sensor values
    float deltaVertical1 = vertical1Raw - prevVertical1;
    float deltaVertical2 = vertical2Raw - prevVertical2;
    float deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
    float deltaHorizontal2 = horizontal2Raw - prevHorizontal2;
    float deltaImu = imuRaw - prevImu;

    // Encoder sanity check: reject deltas that are impossibly large
    // (e.g. from a disconnected sensor returning 0 or PROS_ERR)
    deltaVertical1 = clampMagnitude(deltaVertical1, maxDeltaPerUpdate);
    deltaVertical2 = clampMagnitude(deltaVertical2, maxDeltaPerUpdate);
    deltaHorizontal1 = clampMagnitude(deltaHorizontal1, maxDeltaPerUpdate);
    deltaHorizontal2 = clampMagnitude(deltaHorizontal2, maxDeltaPerUpdate);

    // update the previous sensor values
    prevVertical1 = vertical1Raw;
    prevVertical2 = vertical2Raw;
    prevHorizontal1 = horizontal1Raw;
    prevHorizontal2 = horizontal2Raw;
    prevImu = imuRaw;

    const uint32_t newSeq = odomDeltaSeq.fetch_add(1) + 1;

    odomStateMutex.take();

    // calculate the heading of the robot
    // Priority:
    // 1. Horizontal tracking wheels
    // 2. Vertical tracking wheels
    // 3. Inertial Sensor
    // 4. Drivetrain
    const float headingBefore = odomPose.theta;
    float heading = odomPose.theta;
    const bool usedHorizontalHeading = tryHeadingFromPair(deltaHorizontal1, deltaHorizontal2, odomSensors.horizontal1,
                                                          odomSensors.horizontal2, heading);
    const bool canUseVerticalHeading = odomSensors.vertical1 != nullptr && odomSensors.vertical2 != nullptr &&
                                       !odomSensors.vertical1->getType() && !odomSensors.vertical2->getType();
    bool usedVerticalHeading = false;
    bool usedImuHeading = false;
    bool usedHeadingFallback = false;
    if (!usedHorizontalHeading) {
        usedVerticalHeading =
            canUseVerticalHeading &&
            tryHeadingFromPair(deltaVertical1, deltaVertical2, odomSensors.vertical1, odomSensors.vertical2, heading);
        if (!usedVerticalHeading) {
            if (odomSensors.imu != nullptr && std::isfinite(deltaImu)) {
                heading += deltaImu;
                usedImuHeading = true;
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
    deltaX = clampMagnitude(deltaX, maxDeltaPerUpdate);
    deltaY = clampMagnitude(deltaY, maxDeltaPerUpdate);
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
