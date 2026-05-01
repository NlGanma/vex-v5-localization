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
constexpr std::uint32_t kMclCorrectionTurnCooldownMs = 5;
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
constexpr float kTwoSensorCorrectionMaxHeadingDeltaDeg = 6.0f;
constexpr float kTwoSensorCorrectionNisScale = 0.5f;

LocalizationConfig config;
bool configured = false;
std::atomic<bool> running {false};
pros::Task* task = nullptr;

EKF ekf;
MCL mcl;
lemlib::Pose fusedPose {0, 0, 0};
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
    resetDebugInfo(pose);
    setFusedPoseInternal(pose);
}

TraceSample convertTraceUnits(TraceSample sample, bool radians) {
    if (radians) return sample;

    sample.odomPose.theta = lemlib::radToDeg(sample.odomPose.theta);
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
    uint32_t turnCorrectionCooldownUntilMs = 0;
    MCLMeasurement lastMeasurement {};
    bool lastSensorsStale = false;
    bool lastCorrectionAccepted = false;
    float lastNis = -1.0f;
    lemlib::Pose lastCandidatePose {0, 0, 0};
    int candidateStableScans = 0;
    bool haveCandidatePose = false;

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
            lastNis = -1.0f;
            candidateStableScans = 0;
            haveCandidatePose = false;
        }

        const auto deltas = lemlib::getOdomDeltasSince(lastSeq);
        lemlib::OdomDelta loopDelta {};
        loopDelta.dt = actualDt;
        loopDelta.seq = lastSeq;
        if (!deltas.empty()) {
            if (lastSeq != 0 && deltas.front().seq > lastSeq + 1) {
                const auto snapshot = lemlib::getOdomSnapshot();
                resetFilters(snapshot.pose);
                lastSeq = snapshot.seq;
                loopDelta.seq = lastSeq;
                lastMeasurement = {};
                lastSensorsStale = false;
                lastCorrectionAccepted = false;
                lastNis = -1.0f;
                candidateStableScans = 0;
                haveCandidatePose = false;
            } else {
                for (const auto& delta : deltas) {
                    ekf.predict(delta);
                    mcl.predict(delta);
                }
                loopDelta = deltas.back();
                lastSeq = deltas.back().seq;
            }
        }

        const float angularRate = (actualDt > 1e-4f) ? std::fabs(loopDelta.deltaTheta) / actualDt : 0.0f;
        if (angularRate > kMclCorrectionMaxTurnRate) {
            turnCorrectionCooldownUntilMs = loopNow + kMclCorrectionTurnCooldownMs;
        }

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
            const bool turnCorrectionSuppressed = now < turnCorrectionCooldownUntilMs;
            bool correctionAccepted = false;
            float nis = -1.0f;
            float consistencyNis = -1.0f;
            float measurementPoseDelta = 0.0f;
            float measurementHeadingDelta = 0.0f;
            const lemlib::Pose ekfPose = ekf.state();
            if (rawMeas.valid && rawMeas.activeSensors >= 2) {
                consistencyNis = ekf.innovationNIS(rawMeas.pose, rawMeas.covariance);
                nis = consistencyNis;
                measurementPoseDelta = rawMeas.pose.distance(ekfPose);
                measurementHeadingDelta =
                    std::fabs(lemlib::radToDeg(lemlib::angleError(rawMeas.pose.theta, ekfPose.theta)));
            }
            if (rawMeas.valid) {
                const bool candidateAgrees =
                    haveCandidatePose && rawMeas.pose.distance(lastCandidatePose) <= kMclCandidateAgreementXY &&
                    std::fabs(lemlib::radToDeg(lemlib::angleError(rawMeas.pose.theta, lastCandidatePose.theta))) <=
                        kMclCandidateAgreementHeadingDeg;
                candidateStableScans = candidateAgrees ? candidateStableScans + 1 : 1;
                lastCandidatePose = rawMeas.pose;
                haveCandidatePose = true;
            } else {
                candidateStableScans = 0;
                haveCandidatePose = false;
            }
            const bool threeSensorCorrectionReady = rawMeas.valid && rawMeas.activeSensors >= 3 &&
                                                    rawMeas.confidence >= config.fusion.minConfidence &&
                                                    rawMeas.covariance.m[0][0] <= config.fusion.maxVarXY &&
                                                    rawMeas.covariance.m[1][1] <= config.fusion.maxVarXY &&
                                                    rawMeas.covariance.m[2][2] <= config.fusion.maxVarTheta;
            const bool twoSensorTightReady = rawMeas.valid && rawMeas.activeSensors == 2 &&
                                             rawMeas.confidence >= kTwoSensorCorrectionMinConfidence &&
                                             rawMeas.covariance.m[0][0] <= kTwoSensorCorrectionMaxVarXY &&
                                             rawMeas.covariance.m[1][1] <= kTwoSensorCorrectionMaxVarXY &&
                                             rawMeas.covariance.m[2][2] <= kTwoSensorCorrectionMaxVarTheta;
            const bool twoSensorRelaxedReady = rawMeas.valid && rawMeas.activeSensors == 2 &&
                                               rawMeas.confidence >= kTwoSensorCorrectionRelaxedMinConfidence &&
                                               measurementPoseDelta <= kTwoSensorCorrectionMaxPoseDeltaIn &&
                                               measurementHeadingDelta <= kTwoSensorCorrectionMaxHeadingDeltaDeg &&
                                               rawMeas.covariance.m[0][0] <= kTwoSensorCorrectionRelaxedMaxVarXY &&
                                               rawMeas.covariance.m[1][1] <= kTwoSensorCorrectionRelaxedMaxVarXY &&
                                               rawMeas.covariance.m[2][2] <= kTwoSensorCorrectionRelaxedMaxVarTheta;
            const bool twoSensorCorrectionReady = rawMeas.valid && rawMeas.activeSensors == 2 &&
                                                  angularRate <= kTwoSensorCorrectionMaxTurnRate &&
                                                  (twoSensorTightReady || twoSensorRelaxedReady);
            const int requiredStableScans =
                rawMeas.activeSensors >= 3 ? kMclThreeSensorStableScans : kMclTwoSensorStableScans;
            const bool correctionGeometryReady = rawMeas.activeSensors >= config.fusion.minCorrectionSensors;
            const bool correctionStable = candidateStableScans >= requiredStableScans;
            const bool correctionCloseToTrack =
                measurementPoseDelta <= config.fusion.maxMeasurementDeltaXY &&
                measurementHeadingDelta <= lemlib::radToDeg(config.fusion.maxMeasurementDeltaTheta);

            if (!turnCorrectionSuppressed && !sensorsStale && correctionGeometryReady && correctionStable &&
                correctionCloseToTrack && (threeSensorCorrectionReady || twoSensorCorrectionReady)) {
                const float nisGate = threeSensorCorrectionReady ? config.fusion.nisGate
                                                                 : config.fusion.nisGate * kTwoSensorCorrectionNisScale;
                if (nis <= nisGate) {
                    ekf.update(rawMeas.pose, rawMeas.covariance);
                    correctionAccepted = true;
                }
            }

            if (!correctionAccepted && rawMeas.valid && rawMeas.activeSensors >= 2 &&
                rawMeas.confidence >= kMclRecoveryMinConfidence &&
                (consistencyNis > config.fusion.nisGate * kMclRecoveryNisMultiplier ||
                 (((measurementPoseDelta > kMclRecoveryMaxPoseDeltaIn) ||
                   (measurementHeadingDelta > kMclRecoveryMaxHeadingDeltaDeg)) &&
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
            lastNis = nis;

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
            debugMutex.give();
            lastMclMs = now;
        }

        // Inject fused pose into LemLib, with dt-scaled correction limits
        lemlib::Pose fused = ekf.state();
        setFusedPoseInternal(fused);
        resetMutex.take();
        const bool resetQueued = pendingReset;
        resetMutex.give();
        const auto snapshot = lemlib::getOdomSnapshot();
        lemlib::Pose appliedPose = snapshot.pose;
        if (!resetQueued) {
            if (snapshot.seq == lastSeq) {
                const lemlib::Pose safe =
                    applyCorrectionLimit(snapshot.pose, fused, config.fusion.maxCorrectionXY * dtScale,
                                         config.fusion.maxCorrectionTheta * dtScale, config.fusion.blend);
                lemlib::detail::setPoseSilent(safe, true);
                appliedPose = safe;
            }
        }

        TraceSample sample {};
        sample.timeMs = now;
        sample.odomSeq = snapshot.seq;
        sample.odomDelta = loopDelta;
        sample.odomTelemetry = lemlib::getOdomTelemetryForSeq(snapshot.seq);
        sample.odomPose = snapshot.pose;
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
        for (size_t i = 0; i < sample.sensors.size(); ++i) {
            sample.sensors[i].distance = lastMeasurement.sensorReadings[i].distance;
            sample.sensors[i].confidence = lastMeasurement.sensorReadings[i].confidence;
            sample.sensors[i].inRange = lastMeasurement.sensorReadings[i].inRange;
            sample.sensors[i].confidenceAccepted = lastMeasurement.sensorReadings[i].confidenceAccepted;
            sample.sensors[i].used = lastMeasurement.sensorReadings[i].used;
        }
        recordTraceSample(sample);

        pros::delay(config.fusion.ekfPeriodMs);
    }
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
    if (!configured || running.load()) return;
    running.store(true);
    resetMutex.take();
    pendingReset = false;
    pendingResetSeq = 0;
    resetMutex.give();

    const auto snapshot = lemlib::getOdomSnapshot();
    initialSeq = snapshot.seq;
    resetFilters(snapshot.pose);

    task = new pros::Task(localizationTask);
}

void stop() {
    if (!running.load()) return;
    running.store(false);
    resetMutex.take();
    pendingReset = false;
    pendingResetSeq = 0;
    resetMutex.give();
    if (task != nullptr) {
        // Wait long enough for the task loop to observe running==false and exit.
        // The loop delay is ekfPeriodMs (default 10ms), so 2x is a safe margin.
        pros::delay(config.fusion.ekfPeriodMs * 2 + 10);
        delete task;
        task = nullptr;
    }
}

bool isRunning() { return running.load(); }

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
