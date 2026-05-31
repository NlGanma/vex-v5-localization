#include "lemlib/localization/localization.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>

#include "lemlib/chassis/odom.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"

namespace lemlib::localization {
namespace {
constexpr float kMinDtScale = 0.5f;
constexpr float kMaxDtScale = 2.0f;
constexpr size_t kTraceCapacity = 8192;
constexpr float kMclRecoveryNisMultiplier = 2.0f;
constexpr float kMclRecoveryMinConfidence = 0.45f;
constexpr float kMclRecoveryMaxPoseDeltaIn = 5.0f;
constexpr float kMclRecoveryMaxHeadingDeltaDeg = 12.0f;
constexpr float kMclRecoveryMinCombinedVarXY = 32.0f;
constexpr float kMclCorrectionMaxTurnRate = 4.36332312999f; // rad/s, about 250 deg/s
constexpr int kMclThreeSensorStableScans = 2;
constexpr int kMclTwoSensorStableScans = 4;
constexpr float kMclCandidateAgreementXY = 1.25f;
constexpr float kMclCandidateAgreementHeadingDeg = 2.5f;
constexpr float kTwoSensorCorrectionMinConfidence = 0.60f;
constexpr float kTwoSensorCorrectionRelaxedMinConfidence = 0.50f;
constexpr float kTwoSensorCorrectionMaxTurnRate =
    3.14159265359f; // rad/s, allow moderate heading settle during fast moves
constexpr float kTwoSensorCorrectionMaxVarXY = 16.0f;
constexpr float kTwoSensorCorrectionMaxVarTheta = 0.02f;
constexpr float kTwoSensorCorrectionRelaxedMaxVarXY = 36.0f;
constexpr float kTwoSensorCorrectionRelaxedMaxVarTheta = 0.03f;
constexpr float kTwoSensorCorrectionMaxPoseDeltaIn = 4.0f;
constexpr float kTwoSensorCorrectionNisScale = 0.5f;
constexpr float kLocalRangeCorrectionMaxPoseDeltaIn = 4.0f;
constexpr float kLocalRangeCorrectionMaxInlierMeanResidual = 2.5f;
constexpr float kLocalRangeCorrectionMaxInlierResidual = 4.0f;
constexpr float kLocalRangeCorrectionPriorWeight = 0.12f;
constexpr float kLocalRangeCorrectionInlierReward = 12.0f;
constexpr float kPlanarRangeHeadingVariance = 1.0f; // rad^2; continuous range updates should not steer IMU heading
constexpr bool kAllowTwoSensorContinuousCorrections = false; // latest turn-sweep logs show 2-ray updates bias XY

enum CorrectionRejectMask : uint32_t {
    kRejectTurnSuppressed = 1u << 0,
    kRejectSensorsStale = 1u << 1,
    kRejectNotEnoughSensors = 1u << 2,
    kRejectUnstableCandidate = 1u << 3,
    kRejectPoseDelta = 1u << 4,
    kRejectReadiness = 1u << 5,
    kRejectNis = 1u << 6,
    kRejectNoMeasurement = 1u << 7,
    kRejectSensorResidual = 1u << 8,
    kRejectMotionSuppressed = 1u << 9,
};

LocalizationConfig config;
bool configured = false;
std::atomic<bool> running {false};
std::atomic<bool> motionCorrectionSuppressed {false};
std::atomic<bool> taskExited {false};
pros::Task* task = nullptr;

EKF ekf;
MCL mcl;
lemlib::Pose fusedPose {0, 0, 0};
lemlib::Pose odomOnlyPose {0, 0, 0};
pros::Mutex poseMutex;
DebugInfo debugInfo {};
pros::Mutex debugMutex;
pros::Mutex resetMutex;
bool pendingReset = false;
lemlib::Pose pendingResetPose {0, 0, 0};
uint32_t pendingResetSeq = 0;
uint32_t initialSeq = 0;
pros::Mutex traceMutex;
std::deque<TraceSample> traceSamples;
TraceSample latestTraceSample {};
bool traceEnabled = false;

Mat3 makeInitialCovariance() {
    Mat3 P0 = Mat3::identity(1.0f);
    P0.m[0][0] = config.fusion.initStdXY * config.fusion.initStdXY;
    P0.m[1][1] = config.fusion.initStdXY * config.fusion.initStdXY;
    P0.m[2][2] = config.fusion.initStdTheta * config.fusion.initStdTheta;
    return P0;
}

void stripHeadingFromMeasurement(lemlib::Pose& pose, Mat3& covariance, float trustedHeading) {
    pose.theta = trustedHeading;
    covariance.m[0][2] = 0.0f;
    covariance.m[1][2] = 0.0f;
    covariance.m[2][0] = 0.0f;
    covariance.m[2][1] = 0.0f;
    covariance.m[2][2] = std::max(covariance.m[2][2], kPlanarRangeHeadingVariance);
}

lemlib::Pose predictPoseOnly(lemlib::Pose pose, const lemlib::OdomDelta& delta) {
    const float dx = delta.localX;
    const float dy = delta.localY;
    const float dtheta = delta.deltaTheta;
    const float avgHeading = pose.theta + dtheta * 0.5f;

    pose.x += dy * std::sin(avgHeading) + dx * -std::cos(avgHeading);
    pose.y += dy * std::cos(avgHeading) + dx * std::sin(avgHeading);
    pose.theta = wrapAngle(pose.theta + dtheta);
    return pose;
}

void setFusedPoseInternal(const lemlib::Pose& pose) {
    poseMutex.take();
    fusedPose = pose;
    poseMutex.give();

    debugMutex.take();
    debugInfo.fusedPose = pose;
    debugMutex.give();
}

void resetDebugInfo(const lemlib::Pose& pose) {
    debugMutex.take();
    debugInfo = {};
    debugInfo.fusedPose = pose;
    debugInfo.mclPose = pose;
    debugMutex.give();
}

void resetFilters(const lemlib::Pose& pose) {
    ekf.reset(pose, makeInitialCovariance());
    mcl.reset(pose);
    odomOnlyPose = pose;
    resetDebugInfo(pose);
    setFusedPoseInternal(pose);
}

TraceSample convertTraceUnits(TraceSample sample, bool radians) {
    if (radians) return sample;

    sample.odomPose.theta = lemlib::radToDeg(sample.odomPose.theta);
    sample.odomOnlyPose.theta = lemlib::radToDeg(sample.odomOnlyPose.theta);
    sample.ekfPose.theta = lemlib::radToDeg(sample.ekfPose.theta);
    sample.appliedPose.theta = lemlib::radToDeg(sample.appliedPose.theta);
    sample.mclPose.theta = lemlib::radToDeg(sample.mclPose.theta);
    sample.odomDelta.deltaTheta = lemlib::radToDeg(sample.odomDelta.deltaTheta);
    sample.odomTelemetry.headingBefore = lemlib::radToDeg(sample.odomTelemetry.headingBefore);
    sample.odomTelemetry.headingAfter = lemlib::radToDeg(sample.odomTelemetry.headingAfter);
    sample.odomTelemetry.deltaHeading = lemlib::radToDeg(sample.odomTelemetry.deltaHeading);
    sample.odomTelemetry.imuRaw = lemlib::radToDeg(sample.odomTelemetry.imuRaw);
    sample.odomTelemetry.deltaImu = lemlib::radToDeg(sample.odomTelemetry.deltaImu);
    sample.targetCorrectionTheta = lemlib::radToDeg(sample.targetCorrectionTheta);
    sample.appliedCorrectionTheta = lemlib::radToDeg(sample.appliedCorrectionTheta);
    return sample;
}

void recordTraceSample(const TraceSample& sample) {
    traceMutex.take();
    latestTraceSample = sample;
    if (traceEnabled) {
        if (traceSamples.size() >= kTraceCapacity) traceSamples.pop_front();
        traceSamples.push_back(sample);
    }
    traceMutex.give();
}

float raycastCircle(float sx, float sy, float dirX, float dirY, const FieldConfig::Obstacle& obstacle) {
    const float dx = sx - obstacle.x;
    const float dy = sy - obstacle.y;
    const float a = dirX * dirX + dirY * dirY;
    const float b = 2.0f * (dx * dirX + dy * dirY);
    const float c = dx * dx + dy * dy - obstacle.radius * obstacle.radius;
    const float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) return -1.0f;

    const float sqrtDisc = std::sqrt(disc);
    const float inv2A = 1.0f / (2.0f * a);
    const float t1 = (-b - sqrtDisc) * inv2A;
    const float t2 = (-b + sqrtDisc) * inv2A;
    if (t1 > 0.0f && t2 > 0.0f) return std::min(t1, t2);
    if (t1 > 0.0f) return t1;
    if (t2 > 0.0f) return t2;
    return -1.0f;
}

float raycastRect(float sx, float sy, float dirX, float dirY, const FieldConfig::Obstacle& obstacle) {
    const float minX = obstacle.x - obstacle.halfW;
    const float maxX = obstacle.x + obstacle.halfW;
    const float minY = obstacle.y - obstacle.halfH;
    const float maxY = obstacle.y + obstacle.halfH;
    constexpr float eps = 1e-6f;
    float tMin = -1e9f;
    float tMax = 1e9f;

    if (std::fabs(dirX) < eps) {
        if (sx < minX || sx > maxX) return -1.0f;
    } else {
        const float tx1 = (minX - sx) / dirX;
        const float tx2 = (maxX - sx) / dirX;
        tMin = std::max(tMin, std::min(tx1, tx2));
        tMax = std::min(tMax, std::max(tx1, tx2));
    }

    if (std::fabs(dirY) < eps) {
        if (sy < minY || sy > maxY) return -1.0f;
    } else {
        const float ty1 = (minY - sy) / dirY;
        const float ty2 = (maxY - sy) / dirY;
        tMin = std::max(tMin, std::min(ty1, ty2));
        tMax = std::min(tMax, std::max(ty1, ty2));
    }

    if (tMax < 0.0f || tMin > tMax) return -1.0f;
    if (tMin >= 0.0f) return tMin;
    if (tMax >= 0.0f) return tMax;
    return -1.0f;
}

float expectedDistanceFromPose(const lemlib::Pose& pose, const SensorConfig& sensor) {
    const float sinTheta = std::sin(pose.theta);
    const float cosTheta = std::cos(pose.theta);
    const float sx = pose.x + sensor.dx * cosTheta + sensor.dy * sinTheta;
    const float sy = pose.y - sensor.dx * sinTheta + sensor.dy * cosTheta;
    const float heading = wrapAngle(pose.theta + sensor.dtheta);
    const float dirX = std::sin(heading);
    const float dirY = std::cos(heading);
    constexpr float eps = 1e-6f;
    float best = 1e9f;

    if (std::fabs(dirX) > eps) {
        const float t1 = (config.field.minX - sx) / dirX;
        const float y1 = sy + t1 * dirY;
        if (t1 > 0.0f && y1 >= config.field.minY && y1 <= config.field.maxY) best = std::min(best, t1);

        const float t2 = (config.field.maxX - sx) / dirX;
        const float y2 = sy + t2 * dirY;
        if (t2 > 0.0f && y2 >= config.field.minY && y2 <= config.field.maxY) best = std::min(best, t2);
    }

    if (std::fabs(dirY) > eps) {
        const float t3 = (config.field.minY - sy) / dirY;
        const float x3 = sx + t3 * dirX;
        if (t3 > 0.0f && x3 >= config.field.minX && x3 <= config.field.maxX) best = std::min(best, t3);

        const float t4 = (config.field.maxY - sy) / dirY;
        const float x4 = sx + t4 * dirX;
        if (t4 > 0.0f && x4 >= config.field.minX && x4 <= config.field.maxX) best = std::min(best, t4);
    }

    for (const auto& obstacle : config.field.obstacles) {
        const float hit = obstacle.type == FieldConfig::Obstacle::Type::Circle
                              ? raycastCircle(sx, sy, dirX, dirY, obstacle)
                              : raycastRect(sx, sy, dirX, dirY, obstacle);
        if (hit > 0.0f) best = std::min(best, hit);
    }

    return best >= 1e8f ? -1.0f : best;
}

struct CandidateSensorFit {
    int usableSensors = 0;
    int inlierSensors = 0;
    float meanResidual = 0.0f;
    float maxResidual = 0.0f;
    float inlierMeanResidual = 0.0f;
    float inlierMaxResidual = 0.0f;
};

CandidateSensorFit evaluateCandidateSensorFit(const lemlib::Pose& pose, const MCLMeasurement& measurement) {
    CandidateSensorFit fit {};
    float residualSum = 0.0f;
    float inlierResidualSum = 0.0f;
    for (size_t i = 0; i < config.sensors.size(); ++i) {
        const auto& reading = measurement.sensorReadings[i];
        if (!reading.used || reading.distance <= 0.0f) continue;
        const float expected = expectedDistanceFromPose(pose, config.sensors[i]);
        if (expected <= 0.0f) continue;

        const float residual = std::fabs(reading.distance - expected);
        residualSum += residual;
        fit.maxResidual = std::max(fit.maxResidual, residual);
        ++fit.usableSensors;
        if (residual <= config.mcl.outlierThreshold) {
            ++fit.inlierSensors;
            inlierResidualSum += residual;
            fit.inlierMaxResidual = std::max(fit.inlierMaxResidual, residual);
        }
    }
    if (fit.usableSensors > 0) fit.meanResidual = residualSum / static_cast<float>(fit.usableSensors);
    if (fit.inlierSensors > 0) fit.inlierMeanResidual = inlierResidualSum / static_cast<float>(fit.inlierSensors);
    return fit;
}

struct LocalRangeCorrection {
    bool valid = false;
    lemlib::Pose pose {0, 0, 0};
    Mat3 covariance = Mat3::identity(1.0f);
    CandidateSensorFit fit {};
    float confidence = 0.0f;
};

float localRangeSolveScore(const lemlib::Pose& pose, const MCLMeasurement& measurement,
                           const lemlib::Pose& anchor, CandidateSensorFit& fit) {
    fit = evaluateCandidateSensorFit(pose, measurement);
    if (fit.inlierSensors < 2) return 1.0e9f;

    float robustResidualCost = 0.0f;
    for (size_t i = 0; i < config.sensors.size(); ++i) {
        const auto& reading = measurement.sensorReadings[i];
        if (!reading.used || reading.distance <= 0.0f) continue;
        const float expected = expectedDistanceFromPose(pose, config.sensors[i]);
        if (expected <= 0.0f) continue;

        const float residual = std::fabs(reading.distance - expected);
        const float clipped = std::min(residual, config.mcl.outlierThreshold);
        robustResidualCost += clipped * clipped;
    }

    const float dx = pose.x - anchor.x;
    const float dy = pose.y - anchor.y;
    return robustResidualCost + kLocalRangeCorrectionPriorWeight * (dx * dx + dy * dy) -
           kLocalRangeCorrectionInlierReward * static_cast<float>(fit.inlierSensors);
}

LocalRangeCorrection solveLocalRangeCorrection(const lemlib::Pose& anchor, const MCLMeasurement& measurement) {
    LocalRangeCorrection out {};
    if (!measurement.valid || measurement.activeSensors < 2) return out;

    lemlib::Pose bestPose = anchor;
    CandidateSensorFit bestFit {};
    float bestScore = localRangeSolveScore(bestPose, measurement, anchor, bestFit);

    for (const float step : {2.0f, 1.0f, 0.5f, 0.25f, 0.125f}) {
        const lemlib::Pose center = bestPose;
        for (int ix = -4; ix <= 4; ++ix) {
            for (int iy = -4; iy <= 4; ++iy) {
                lemlib::Pose candidate = center;
                candidate.x = center.x + static_cast<float>(ix) * step;
                candidate.y = center.y + static_cast<float>(iy) * step;
                candidate.theta = anchor.theta;

                CandidateSensorFit fit {};
                const float score = localRangeSolveScore(candidate, measurement, anchor, fit);
                if (score < bestScore) {
                    bestScore = score;
                    bestPose = candidate;
                    bestFit = fit;
                }
            }
        }
    }

    const float delta = bestPose.distance(anchor);
    if (bestFit.inlierSensors < 2 || delta > kLocalRangeCorrectionMaxPoseDeltaIn ||
        bestFit.inlierMeanResidual > kLocalRangeCorrectionMaxInlierMeanResidual ||
        bestFit.inlierMaxResidual > kLocalRangeCorrectionMaxInlierResidual) {
        return out;
    }

    out.valid = true;
    out.pose = bestPose;
    out.fit = bestFit;
    out.confidence =
        std::clamp(1.0f - bestFit.inlierMeanResidual / std::max(config.mcl.outlierThreshold, 1.0f), 0.1f, 0.95f);
    const float xyStd = std::clamp(std::max(2.0f, bestFit.inlierMaxResidual), 2.0f, 6.0f);
    out.covariance = Mat3::identity(0.0f);
    out.covariance.m[0][0] = xyStd * xyStd;
    out.covariance.m[1][1] = xyStd * xyStd;
    out.covariance.m[2][2] = kPlanarRangeHeadingVariance;
    return out;
}

lemlib::Pose applyCorrectionLimit(const lemlib::Pose& current, const lemlib::Pose& target, float maxXY, float maxTheta,
                                  float blend) {
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    float dtheta = wrapAngle(target.theta - current.theta);

    const float dist = std::hypot(dx, dy);
    if (maxXY <= 0.0f) {
        dx = 0.0f;
        dy = 0.0f;
    } else if (dist > maxXY) {
        const float scale = maxXY / dist;
        dx *= scale;
        dy *= scale;
    }
    if (maxTheta <= 0.0f) dtheta = 0.0f;
    else if (std::fabs(dtheta) > maxTheta) dtheta = std::copysign(maxTheta, dtheta);

    // Keep heading continuous in odom state; only the correction delta should be wrapped.
    lemlib::Pose limited {current.x + dx, current.y + dy, current.theta + dtheta};

    if (blend < 1.0f) {
        limited.x = current.x + blend * (limited.x - current.x);
        limited.y = current.y + blend * (limited.y - current.y);
        limited.theta = current.theta + blend * wrapAngle(limited.theta - current.theta);
    }

    return limited;
}

void localizationTask(void*) {
    uint32_t lastMclMs = pros::millis();
    uint32_t lastSeq = initialSeq;
    uint32_t lastLoopMs = pros::millis();
    uint32_t lastActiveSensorMs = pros::millis(); // sensor staleness watchdog
    // Max instantaneous turn rate seen since the last MCL update. The loop runs
    // every ekfPeriodMs (~10ms) but the MCL turn gate is only read every
    // mclPeriodMs (~50ms); tracking the window max ensures a fast turn between
    // MCL ticks still suppresses corrections (a fixed-ms cooldown shorter than
    // the read cadence could never be observed true).
    float maxAngularRateSinceMcl = 0.0f;
    MCLMeasurement lastMeasurement {};
    bool lastSensorsStale = false;
    bool lastCorrectionAccepted = false;
    uint32_t lastCorrectionRejectMask = 0;
    float lastNis = -1.0f;
    float lastMeasurementPoseDelta = 0.0f;
    float lastMeasurementHeadingDelta = 0.0f;
    int lastCorrectionInlierSensors = 0;
    float lastCorrectionMeanResidual = 0.0f;
    float lastCorrectionMaxResidual = 0.0f;
    int lastCandidateStableScans = 0;
    int lastRequiredStableScans = 0;
    lemlib::Pose lastCandidatePose {0, 0, 0};
    int candidateStableScans = 0;
    bool haveCandidatePose = false;
    std::array<float, 4> lastSensorDistance {};
    std::array<int, 4> lastSensorConfidence {};
    std::array<int, 4> lastSensorObjectSize {};
    std::array<bool, 4> lastSensorUsed {};
    std::array<bool, 4> haveSensorReading {};
    std::array<uint32_t, 4> lastSensorChangeMs {};
    std::array<std::uint16_t, 4> sensorReadingSeq {};
    lastSensorDistance.fill(-1.0f);
    lastSensorConfidence.fill(-1);
    lastSensorObjectSize.fill(-1);

    while (running.load()) {
        const uint32_t loopNow = pros::millis();
        const float actualDt = static_cast<float>(loopNow - lastLoopMs) / 1000.0f;
        const float nominalDt = static_cast<float>(config.fusion.ekfPeriodMs) / 1000.0f;
        // dt scale factor for velocity-based correction limits
        const float dtScale = (nominalDt > 0.0f) ? std::clamp(actualDt / nominalDt, kMinDtScale, kMaxDtScale) : 1.0f;
        lastLoopMs = loopNow;

        resetMutex.take();
        const bool applyReset = pendingReset;
        const lemlib::Pose resetPose = pendingResetPose;
        const uint32_t resetSeq = pendingResetSeq;
        pendingReset = false;
        resetMutex.give();
        if (applyReset) {
            resetFilters(resetPose);
            lastSeq = resetSeq;
            lastMeasurement = {};
            lastSensorsStale = false;
            lastCorrectionAccepted = false;
            lastCorrectionRejectMask = 0;
            lastNis = -1.0f;
            lastMeasurementPoseDelta = 0.0f;
            lastMeasurementHeadingDelta = 0.0f;
            lastCorrectionInlierSensors = 0;
            lastCorrectionMeanResidual = 0.0f;
            lastCorrectionMaxResidual = 0.0f;
            lastCandidateStableScans = 0;
            lastRequiredStableScans = 0;
            candidateStableScans = 0;
            haveCandidatePose = false;
            maxAngularRateSinceMcl = 0.0f;
            haveSensorReading.fill(false);
            sensorReadingSeq.fill(0);
        }

        const auto deltas = lemlib::getOdomDeltasSince(lastSeq);
        lemlib::OdomDelta loopDelta {};
        loopDelta.dt = actualDt;
        loopDelta.seq = lastSeq;
        // Fastest instantaneous turn rate among the deltas replayed this loop
        // (0 when none were applied). Computed per-delta to avoid actualDt dilution.
        float angularRate = 0.0f;
        if (!deltas.empty()) {
            if (lastSeq != 0 && deltas.front().seq > lastSeq + 1) {
                const auto snapshot = lemlib::getOdomSnapshot();
                resetFilters(snapshot.pose);
                lastSeq = snapshot.seq;
                loopDelta.seq = lastSeq;
                lastMeasurement = {};
                lastSensorsStale = false;
                lastCorrectionAccepted = false;
                lastCorrectionRejectMask = 0;
                lastNis = -1.0f;
                lastMeasurementPoseDelta = 0.0f;
                lastMeasurementHeadingDelta = 0.0f;
                lastCorrectionInlierSensors = 0;
                lastCorrectionMeanResidual = 0.0f;
                lastCorrectionMaxResidual = 0.0f;
                lastCandidateStableScans = 0;
                lastRequiredStableScans = 0;
                candidateStableScans = 0;
                haveCandidatePose = false;
                maxAngularRateSinceMcl = 0.0f;
            } else {
                for (const auto& delta : deltas) {
                    ekf.predict(delta);
                    mcl.predict(delta);
                    odomOnlyPose = predictPoseOnly(odomOnlyPose, delta);
                    // Use each delta's own dt so a fast turn batched under
                    // scheduler lag is never diluted by the multi-delta actualDt
                    // window; max keeps the turn gate conservative.
                    if (delta.dt > 1e-4f) {
                        angularRate = std::max(angularRate, std::fabs(delta.deltaTheta) / delta.dt);
                    }
                }
                loopDelta = deltas.back();
                lastSeq = deltas.back().seq;
            }
        }

        maxAngularRateSinceMcl = std::max(maxAngularRateSinceMcl, angularRate);

        const uint32_t now = pros::millis();
        if (now - lastMclMs >= config.fusion.mclPeriodMs) {
            const auto rawMeas = mcl.update();
            MCLMeasurement meas = rawMeas;
            if (rawMeas.activeSensors > 0) lastActiveSensorMs = now;

            // Keep the last trusted particle estimate in debug output when the
            // current scan is unusable, instead of replacing it with a default
            // zero pose that obscures what the particle cloud was doing.
            if (!meas.valid) {
                if (lastMeasurement.valid) {
                    meas.pose = lastMeasurement.pose;
                    meas.covariance = lastMeasurement.covariance;
                } else {
                    meas.pose = ekf.state();
                    meas.covariance = makeInitialCovariance();
                }
                meas.confidence = 0.0f;
            }

            // Skip the EKF correction if the sensors have been stale, but keep
            // sampling so the filter can recover as soon as fresh readings return.
            const bool sensorsStale = (now - lastActiveSensorMs) > config.fusion.sensorStaleMs;
            const bool turnCorrectionSuppressed = maxAngularRateSinceMcl > kMclCorrectionMaxTurnRate;
            bool correctionAccepted = false;
            float nis = -1.0f;
            float consistencyNis = -1.0f;
            float measurementPoseDelta = 0.0f;
            float measurementHeadingDelta = 0.0f;
            const lemlib::Pose ekfPose = ekf.state();
            CandidateSensorFit candidateSensorFit {};
            LocalRangeCorrection localRangeCorrection {};
            lemlib::Pose correctionCandidatePose = rawMeas.pose;
            Mat3 correctionCandidateCovariance = rawMeas.covariance;
            bool usingLocalRangeCorrection = false;
            float rawMclConsistencyNis = -1.0f;
            float rawMclPoseDelta = 0.0f;
            float rawMclHeadingDelta = 0.0f;
            if (rawMeas.valid && rawMeas.activeSensors >= 2) {
                lemlib::Pose nisPose = rawMeas.pose;
                Mat3 nisCovariance = rawMeas.covariance;
                stripHeadingFromMeasurement(nisPose, nisCovariance, ekfPose.theta);
                rawMclConsistencyNis = ekf.innovationNIS(nisPose, nisCovariance);
                rawMclPoseDelta = rawMeas.pose.distance(ekfPose);
                rawMclHeadingDelta =
                    std::fabs(lemlib::radToDeg(lemlib::angleError(rawMeas.pose.theta, ekfPose.theta)));
                candidateSensorFit = evaluateCandidateSensorFit(nisPose, rawMeas);
                correctionCandidatePose = nisPose;
                correctionCandidateCovariance = nisCovariance;
                consistencyNis = rawMclConsistencyNis;
                nis = consistencyNis;
                measurementPoseDelta = correctionCandidatePose.distance(ekfPose);
                measurementHeadingDelta =
                    std::fabs(lemlib::radToDeg(lemlib::angleError(correctionCandidatePose.theta, ekfPose.theta)));

                localRangeCorrection = solveLocalRangeCorrection(ekfPose, rawMeas);
                if (localRangeCorrection.valid) {
                    correctionCandidatePose = localRangeCorrection.pose;
                    correctionCandidateCovariance = localRangeCorrection.covariance;
                    candidateSensorFit = localRangeCorrection.fit;
                    usingLocalRangeCorrection = true;
                    consistencyNis = ekf.innovationNIS(correctionCandidatePose, correctionCandidateCovariance);
                    nis = consistencyNis;
                    measurementPoseDelta = correctionCandidatePose.distance(ekfPose);
                    measurementHeadingDelta = 0.0f;
                }
            }
            if (rawMeas.valid || usingLocalRangeCorrection) {
                // Track stability on a single consistent estimator (the raw MCL
                // weighted mean) regardless of whether the local-range refinement
                // is active this scan. Mixing the two sources let a toggle of
                // usingLocalRangeCorrection masquerade as candidate instability,
                // spuriously resetting candidateStableScans and starving the
                // accept gate.
                const lemlib::Pose stableCandidatePose = rawMeas.pose;
                const bool candidateAgrees =
                    haveCandidatePose && stableCandidatePose.distance(lastCandidatePose) <= kMclCandidateAgreementXY &&
                    std::fabs(lemlib::radToDeg(lemlib::angleError(stableCandidatePose.theta, lastCandidatePose.theta))) <=
                        kMclCandidateAgreementHeadingDeg;
                candidateStableScans = candidateAgrees ? candidateStableScans + 1 : 1;
                lastCandidatePose = stableCandidatePose;
                haveCandidatePose = true;
            } else {
                candidateStableScans = 0;
                haveCandidatePose = false;
            }
            const bool candidateHasThreeInliers =
                candidateSensorFit.inlierSensors >= config.fusion.minCorrectionSensors;
            const bool candidateHasTwoRawInliers =
                rawMeas.valid && rawMeas.activeSensors == 2 && candidateSensorFit.inlierSensors >= 2;
            const bool threeSensorCorrectionReady = rawMeas.valid && rawMeas.activeSensors >= 3 &&
                                                    candidateHasThreeInliers &&
                                                    rawMeas.confidence >= config.fusion.minConfidence &&
                                                    rawMeas.covariance.m[0][0] <= config.fusion.maxVarXY &&
                                                    rawMeas.covariance.m[1][1] <= config.fusion.maxVarXY &&
                                                    rawMeas.covariance.m[2][2] <= config.fusion.maxVarTheta;
            const bool twoSensorTightReady = rawMeas.valid && rawMeas.activeSensors == 2 &&
                                             candidateHasTwoRawInliers &&
                                             rawMeas.confidence >= kTwoSensorCorrectionMinConfidence &&
                                             rawMeas.covariance.m[0][0] <= kTwoSensorCorrectionMaxVarXY &&
                                             rawMeas.covariance.m[1][1] <= kTwoSensorCorrectionMaxVarXY &&
                                             rawMeas.covariance.m[2][2] <= kTwoSensorCorrectionMaxVarTheta;
            const bool twoSensorRelaxedReady = rawMeas.valid && rawMeas.activeSensors == 2 &&
                                               candidateHasTwoRawInliers &&
                                               rawMeas.confidence >= kTwoSensorCorrectionRelaxedMinConfidence &&
                                               measurementPoseDelta <= kTwoSensorCorrectionMaxPoseDeltaIn &&
                                               rawMeas.covariance.m[0][0] <= kTwoSensorCorrectionRelaxedMaxVarXY &&
                                               rawMeas.covariance.m[1][1] <= kTwoSensorCorrectionRelaxedMaxVarXY &&
                                               rawMeas.covariance.m[2][2] <= kTwoSensorCorrectionRelaxedMaxVarTheta;
            const bool twoSensorCorrectionReady = kAllowTwoSensorContinuousCorrections && rawMeas.valid &&
                                                  rawMeas.activeSensors == 2 &&
                                                  maxAngularRateSinceMcl <= kTwoSensorCorrectionMaxTurnRate &&
                                                  (twoSensorTightReady || twoSensorRelaxedReady);
            const bool localRangeCorrectionReady =
                usingLocalRangeCorrection && localRangeCorrection.confidence >= kTwoSensorCorrectionRelaxedMinConfidence &&
                maxAngularRateSinceMcl <= kTwoSensorCorrectionMaxTurnRate;
            const int requiredStableScans =
                candidateSensorFit.inlierSensors >= 3 ? kMclThreeSensorStableScans : kMclTwoSensorStableScans;
            const bool correctionGeometryReady =
                candidateHasThreeInliers || twoSensorCorrectionReady || localRangeCorrectionReady;
            const bool correctionStable = candidateStableScans >= requiredStableScans;
            const bool planarRangeCorrection = rawMeas.valid && rawMeas.activeSensors >= 2;
            const float maxMeasurementDelta =
                localRangeCorrectionReady ? kLocalRangeCorrectionMaxPoseDeltaIn : config.fusion.maxMeasurementDeltaXY;
            const bool correctionCloseToTrack =
                measurementPoseDelta <= maxMeasurementDelta &&
                (planarRangeCorrection ||
                 measurementHeadingDelta <= lemlib::radToDeg(config.fusion.maxMeasurementDeltaTheta));
            const bool correctionReady = threeSensorCorrectionReady || twoSensorCorrectionReady || localRangeCorrectionReady;
            // Full NIS gate only for genuine 3-inlier geometry; a 2-inlier
            // local-range solve is two-sensor-class and gets the tightened gate,
            // matching the requiredStableScans inlier test.
            const float activeNisGate =
                (candidateSensorFit.inlierSensors >= 3)
                    ? config.fusion.nisGate
                    : config.fusion.nisGate * kTwoSensorCorrectionNisScale;
            uint32_t correctionRejectMask = 0;

            if (!rawMeas.valid || rawMeas.activeSensors < 2) correctionRejectMask |= kRejectNoMeasurement;
            if (turnCorrectionSuppressed) correctionRejectMask |= kRejectTurnSuppressed;
            if (sensorsStale) correctionRejectMask |= kRejectSensorsStale;
            const bool motionCorrectionSuppressedForUpdate = motionCorrectionSuppressed.load();
            if (motionCorrectionSuppressedForUpdate) correctionRejectMask |= kRejectMotionSuppressed;
            if (!correctionGeometryReady) correctionRejectMask |= kRejectNotEnoughSensors;
            // Distinct residual-quality signal: enough sensors are live but too
            // many were rejected as outliers to form a full-strength inlier fix.
            // Independent of kRejectNotEnoughSensors so the trace can tell a
            // sensor-count failure apart from a residual failure.
            if (rawMeas.valid && rawMeas.activeSensors >= config.fusion.minCorrectionSensors &&
                candidateSensorFit.inlierSensors < config.fusion.minCorrectionSensors) {
                correctionRejectMask |= kRejectSensorResidual;
            }
            if (!correctionStable) correctionRejectMask |= kRejectUnstableCandidate;
            if (!correctionCloseToTrack) correctionRejectMask |= kRejectPoseDelta;
            if (!correctionReady) correctionRejectMask |= kRejectReadiness;
            if (correctionReady && nis > activeNisGate) correctionRejectMask |= kRejectNis;

            if (!motionCorrectionSuppressedForUpdate && !turnCorrectionSuppressed && !sensorsStale && correctionGeometryReady &&
                correctionStable && correctionCloseToTrack && correctionReady && nis <= activeNisGate) {
                lemlib::Pose correctionPose = correctionCandidatePose;
                Mat3 correctionCovariance = correctionCandidateCovariance;
                const float trustedHeading = ekfPose.theta;
                const float trustedHeadingVariance = ekf.covariance().m[2][2];
                if (planarRangeCorrection && !usingLocalRangeCorrection) {
                    stripHeadingFromMeasurement(correctionPose, correctionCovariance, trustedHeading);
                }

                ekf.update(correctionPose, correctionCovariance);
                if (planarRangeCorrection) {
                    lemlib::Pose planarState = ekf.state();
                    Mat3 planarCovariance = ekf.covariance();
                    planarState.theta = trustedHeading;
                    planarCovariance.m[0][2] = 0.0f;
                    planarCovariance.m[1][2] = 0.0f;
                    planarCovariance.m[2][0] = 0.0f;
                    planarCovariance.m[2][1] = 0.0f;
                    planarCovariance.m[2][2] = trustedHeadingVariance;
                    ekf.reset(planarState, planarCovariance);
                }
                correctionAccepted = true;
                correctionRejectMask = 0;
            }

            if (!correctionAccepted && rawMeas.valid && rawMeas.activeSensors >= 2 &&
                rawMeas.confidence >= kMclRecoveryMinConfidence &&
                (rawMclConsistencyNis > config.fusion.nisGate * kMclRecoveryNisMultiplier ||
                 (((rawMclPoseDelta > kMclRecoveryMaxPoseDeltaIn) ||
                   (rawMclHeadingDelta > kMclRecoveryMaxHeadingDeltaDeg)) &&
                  (rawMeas.covariance.m[0][0] + rawMeas.covariance.m[1][1] >= kMclRecoveryMinCombinedVarXY)))) {
                // If MCL becomes moderately/highly confident in a pose that is
                // still inconsistent with the EKF track, reseed it from the
                // trusted state so the particle cloud does not stay aliased
                // once the late-run two-sensor geometry becomes ambiguous.
                const lemlib::Pose anchor = ekfPose;
                mcl.reset(anchor);
                meas.pose = anchor;
                meas.covariance = makeInitialCovariance();
                meas.confidence = 0.0f;
                meas.valid = false;
                candidateStableScans = 0;
                haveCandidatePose = false;
            }
            lastMeasurement = meas;
            lastSensorsStale = sensorsStale;
            lastCorrectionAccepted = correctionAccepted;
            lastCorrectionRejectMask = correctionRejectMask;
            lastNis = nis;
            lastMeasurementPoseDelta = measurementPoseDelta;
            lastMeasurementHeadingDelta = measurementHeadingDelta;
            lastCorrectionInlierSensors = candidateSensorFit.inlierSensors;
            lastCorrectionMeanResidual = candidateSensorFit.meanResidual;
            lastCorrectionMaxResidual = candidateSensorFit.maxResidual;
            lastCandidateStableScans = candidateStableScans;
            lastRequiredStableScans = requiredStableScans;

            debugMutex.take();
            debugInfo.mclPose = meas.pose;
            debugInfo.mclConfidence = meas.confidence;
            debugInfo.mclEss = meas.ess;
            debugInfo.lastNis = nis;
            debugInfo.mclVarX = meas.covariance.m[0][0];
            debugInfo.mclVarY = meas.covariance.m[1][1];
            debugInfo.mclVarTheta = meas.covariance.m[2][2];
            debugInfo.activeSensors = meas.activeSensors;
            debugInfo.mclValid = meas.valid;
            debugInfo.sensorsStale = lastSensorsStale;
            debugInfo.correctionAccepted = lastCorrectionAccepted;
            debugInfo.correctionRejectMask = lastCorrectionRejectMask;
            debugInfo.measurementPoseDelta = lastMeasurementPoseDelta;
            debugInfo.measurementHeadingDelta = lastMeasurementHeadingDelta;
            debugInfo.correctionInlierSensors = lastCorrectionInlierSensors;
            debugInfo.correctionMeanResidual = lastCorrectionMeanResidual;
            debugInfo.correctionMaxResidual = lastCorrectionMaxResidual;
            debugInfo.candidateStableScans = lastCandidateStableScans;
            debugInfo.requiredStableScans = lastRequiredStableScans;
            debugMutex.give();
            lastMclMs = now;
            maxAngularRateSinceMcl = 0.0f; // window consumed; restart turn-rate tracking
        }

        // Inject fused pose into LemLib, with dt-scaled correction limits
        lemlib::Pose fused = ekf.state();
        setFusedPoseInternal(fused);
        resetMutex.take();
        const bool resetQueued = pendingReset;
        resetMutex.give();
        const auto snapshot = lemlib::getOdomSnapshot();
        lemlib::Pose appliedPose = snapshot.pose;
        const bool motionCorrectionSuppressedForInjection = motionCorrectionSuppressed.load();
        if (!resetQueued && !motionCorrectionSuppressedForInjection) {
            // Only write the correction back when the published odom snapshot is
            // exactly the state our fused pose was built from (seq == lastSeq). If
            // odom advanced mid-loop, the correction would be relative to a stale
            // baseline, so we defer it rather than apply a skewed delta. This
            // intentionally throttles correction cadence (a tuning/perf lever, not
            // a bug); EKF state persists, so correction authority is not lost.
            if (snapshot.seq == lastSeq) {
                const lemlib::Pose safe =
                    applyCorrectionLimit(snapshot.pose, fused, config.fusion.maxCorrectionXY * dtScale,
                                         config.fusion.maxCorrectionTheta * dtScale, config.fusion.blend);
                // Commit only if odom has not advanced since the snapshot: the
                // setter re-checks the seq under the odom lock, so a concurrent
                // odom integration can't be clobbered (correction is simply
                // deferred to the next loop instead).
                if (lemlib::detail::setPoseSilentIfSeq(safe, snapshot.seq, true)) appliedPose = safe;
            }
        }

        TraceSample sample {};
        sample.timeMs = now;
        sample.odomSeq = snapshot.seq;
        sample.odomDelta = loopDelta;
        sample.odomTelemetry = lemlib::getOdomTelemetryForSeq(snapshot.seq);
        sample.odomPose = snapshot.pose;
        sample.odomOnlyPose = odomOnlyPose;
        sample.ekfPose = fused;
        sample.appliedPose = appliedPose;
        sample.mclPose = lastMeasurement.pose;
        sample.targetCorrectionX = fused.x - snapshot.pose.x;
        sample.targetCorrectionY = fused.y - snapshot.pose.y;
        sample.targetCorrectionTheta = wrapAngle(fused.theta - snapshot.pose.theta);
        sample.appliedCorrectionX = appliedPose.x - snapshot.pose.x;
        sample.appliedCorrectionY = appliedPose.y - snapshot.pose.y;
        sample.appliedCorrectionTheta = wrapAngle(appliedPose.theta - snapshot.pose.theta);
        sample.mclConfidence = lastMeasurement.confidence;
        sample.mclEss = lastMeasurement.ess;
        sample.lastNis = lastNis;
        sample.mclVarX = lastMeasurement.covariance.m[0][0];
        sample.mclVarY = lastMeasurement.covariance.m[1][1];
        sample.mclVarTheta = lastMeasurement.covariance.m[2][2];
        sample.activeSensors = lastMeasurement.activeSensors;
        sample.mclValid = lastMeasurement.valid;
        sample.sensorsStale = lastSensorsStale;
        sample.correctionAccepted = lastCorrectionAccepted;
        sample.correctionRejectMask = lastCorrectionRejectMask;
        sample.measurementPoseDelta = lastMeasurementPoseDelta;
        sample.measurementHeadingDelta = lastMeasurementHeadingDelta;
        sample.correctionInlierSensors = lastCorrectionInlierSensors;
        sample.correctionMeanResidual = lastCorrectionMeanResidual;
        sample.correctionMaxResidual = lastCorrectionMaxResidual;
        sample.candidateStableScans = lastCandidateStableScans;
        sample.requiredStableScans = lastRequiredStableScans;
        for (size_t i = 0; i < sample.sensors.size(); ++i) {
            auto& sensorTrace = sample.sensors[i];
            const auto& sensorDebug = lastMeasurement.sensorReadings[i];
            sensorTrace.distance = sensorDebug.distance;
            sensorTrace.objectVelocity = sensorDebug.objectVelocity;
            sensorTrace.confidence = sensorDebug.confidence;
            sensorTrace.objectSize = sensorDebug.objectSize;
            sensorTrace.inRange = sensorDebug.inRange;
            sensorTrace.confidenceAccepted = sensorDebug.confidenceAccepted;
            sensorTrace.used = sensorDebug.used;
            sensorTrace.expectedDistance = expectedDistanceFromPose(sample.appliedPose, config.sensors[i]);
            sensorTrace.residual = (sensorTrace.distance > 0.0f && sensorTrace.expectedDistance > 0.0f)
                                       ? sensorTrace.distance - sensorTrace.expectedDistance
                                       : 0.0f;

            const bool readingChanged =
                !haveSensorReading[i] || sensorTrace.distance != lastSensorDistance[i] ||
                sensorTrace.confidence != lastSensorConfidence[i] || sensorTrace.objectSize != lastSensorObjectSize[i] ||
                sensorTrace.used != lastSensorUsed[i];
            if (readingChanged) {
                haveSensorReading[i] = true;
                lastSensorDistance[i] = sensorTrace.distance;
                lastSensorConfidence[i] = sensorTrace.confidence;
                lastSensorObjectSize[i] = sensorTrace.objectSize;
                lastSensorUsed[i] = sensorTrace.used;
                lastSensorChangeMs[i] = sample.timeMs;
                if (sensorReadingSeq[i] < UINT16_MAX) ++sensorReadingSeq[i];
            }
            const uint32_t ageMs = sample.timeMs - lastSensorChangeMs[i];
            sensorTrace.readingAgeMs = static_cast<std::uint16_t>(std::min<uint32_t>(ageMs, UINT16_MAX));
            sensorTrace.readingSeq = sensorReadingSeq[i];
            sensorTrace.readingChanged = readingChanged;
        }
        recordTraceSample(sample);

        pros::delay(config.fusion.ekfPeriodMs);
    }
    taskExited.store(true); // handshake: stop() waits on this before delete
}
} // namespace

void configure(const LocalizationConfig& cfg) {
    const bool shouldRestart = running.load();
    if (shouldRestart) stop();

    config = cfg;
    config.fusion.ekfPeriodMs = std::max(config.fusion.ekfPeriodMs, static_cast<uint32_t>(1));
    config.fusion.mclPeriodMs = std::max(config.fusion.mclPeriodMs, static_cast<uint32_t>(1));
    config.fusion.sensorStaleMs = std::max(config.fusion.sensorStaleMs, config.fusion.mclPeriodMs);
    config.fusion.minCorrectionSensors = std::clamp(config.fusion.minCorrectionSensors, 1, 4);
    config.fusion.blend = std::clamp(config.fusion.blend, 0.0f, 1.0f);
    config.fusion.minConfidence = std::clamp(config.fusion.minConfidence, 0.0f, 1.0f);
    config.fusion.maxVarXY = std::max(config.fusion.maxVarXY, 1e-6f);
    config.fusion.maxVarTheta = std::max(config.fusion.maxVarTheta, 1e-6f);
    config.fusion.maxMeasurementDeltaXY = std::fabs(config.fusion.maxMeasurementDeltaXY);
    config.fusion.maxMeasurementDeltaTheta = std::fabs(config.fusion.maxMeasurementDeltaTheta);
    config.fusion.maxCorrectionXY = std::fabs(config.fusion.maxCorrectionXY);
    config.fusion.maxCorrectionTheta = std::fabs(config.fusion.maxCorrectionTheta);
    config.fusion.initStdXY = std::max(config.fusion.initStdXY, 1e-3f);
    config.fusion.initStdTheta = std::max(config.fusion.initStdTheta, 1e-3f);
    config.fusion.nisGate = std::max(config.fusion.nisGate, 0.0f);
    configured = true;
    ekf.setConfig(config.ekf);
    mcl.configure(config.field, config.mcl, config.sensors);
    if (shouldRestart) start();
}

void start() {
    if (!configured) return;
    // Atomic check-then-act: only one caller may flip running false->true and
    // create the task. A non-atomic load/store could let two threads both pass
    // the guard, spawn two localizationTask threads racing on the unguarded
    // ekf/mcl globals, and leak the first Task.
    bool expected = false;
    if (!running.compare_exchange_strong(expected, true)) return;
    motionCorrectionSuppressed.store(false);
    taskExited.store(false);
    resetMutex.take();
    pendingReset = false;
    pendingResetSeq = 0;
    resetMutex.give();

    const auto snapshot = lemlib::getOdomSnapshot();
    initialSeq = snapshot.seq;
    resetFilters(snapshot.pose);

    if (task == nullptr) task = new pros::Task(localizationTask);
}

void stop() {
    if (!running.load()) return;
    running.store(false);
    motionCorrectionSuppressed.store(false);
    resetMutex.take();
    pendingReset = false;
    pendingResetSeq = 0;
    resetMutex.give();
    if (task != nullptr) {
        // Real handshake: wait until the task loop observes running==false and
        // sets taskExited just before returning, so we never delete it mid-body
        // (which could orphan a held debug/trace mutex). Bounded by a timeout.
        uint32_t waitedMs = 0;
        const uint32_t maxWaitMs = config.fusion.ekfPeriodMs * 20 + 50;
        while (!taskExited.load() && waitedMs < maxWaitMs) {
            pros::delay(5);
            waitedMs += 5;
        }
        delete task;
        task = nullptr;
    }
}

bool isRunning() { return running.load(); }

void setMotionCorrectionSuppressed(bool suppressed) { motionCorrectionSuppressed.store(suppressed); }

bool isMotionCorrectionSuppressed() { return motionCorrectionSuppressed.load(); }

void syncPose(lemlib::Pose pose) { syncPose(pose, lemlib::getOdomSnapshot().seq); }

void syncPose(lemlib::Pose pose, uint32_t seq) {
    if (!configured) return;

    if (!running.load()) {
        initialSeq = seq;
        resetFilters(pose);
        return;
    }

    resetMutex.take();
    pendingResetPose = pose;
    pendingResetSeq = seq;
    pendingReset = true;
    resetMutex.give();
    setFusedPoseInternal(pose);
}

lemlib::Pose getFusedPose(bool radians) {
    poseMutex.take();
    const lemlib::Pose pose = fusedPose;
    poseMutex.give();
    if (radians) return pose;
    return lemlib::Pose(pose.x, pose.y, lemlib::radToDeg(pose.theta));
}

DebugInfo getDebugInfo(bool radians) {
    debugMutex.take();
    DebugInfo info = debugInfo;
    debugMutex.give();
    if (!radians) {
        info.fusedPose.theta = lemlib::radToDeg(info.fusedPose.theta);
        info.mclPose.theta = lemlib::radToDeg(info.mclPose.theta);
    }
    return info;
}

LocalizationConfig getConfig() { return config; }

void setTraceEnabled(bool enabled) {
    traceMutex.take();
    traceEnabled = enabled;
    traceMutex.give();
}

void clearTrace() {
    traceMutex.take();
    traceSamples.clear();
    latestTraceSample = {};
    traceMutex.give();
}

TraceSample getLatestTraceSample(bool radians) {
    traceMutex.take();
    TraceSample sample = latestTraceSample;
    traceMutex.give();
    return convertTraceUnits(sample, radians);
}

std::vector<TraceSample> getTraceSamples(bool radians) {
    traceMutex.take();
    std::vector<TraceSample> samples(traceSamples.begin(), traceSamples.end());
    traceMutex.give();
    if (radians) return samples;
    for (TraceSample& sample : samples) sample = convertTraceUnits(sample, false);
    return samples;
}
} // namespace lemlib::localization
