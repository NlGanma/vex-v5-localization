#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "lemlib/chassis/odom.hpp"
#include "lemlib/localization/ekf.hpp"
#include "lemlib/localization/mcl.hpp"
#include "lemlib/pose.hpp"

namespace lemlib::localization {
struct FusionConfig { // TUNE
        uint32_t ekfPeriodMs = 10;
        uint32_t mclPeriodMs = 50;
        float nisGate = 9.21f; // chi-square(3) ~97.5%
        int minCorrectionSensors = 3; // continuous EKF corrections need this many live range sensors
        float minConfidence = 0.4f;
        float maxVarXY = 36.0f; // inches^2
        float maxVarTheta = 0.12f; // rad^2
        float maxMeasurementDeltaXY = 3.0f; // inches, reject MCL poses this far from odom/EKF track
        float maxMeasurementDeltaTheta = 0.0872665f; // radians (~5 deg)
        float maxCorrectionXY = 0.5f; // inches per update
        float maxCorrectionTheta = 0.1f; // radians per update
        float blend = 1.0f; // 1 = no extra smoothing, <1 adds smoothing
        float initStdXY = 3.0f; // inches
        float initStdTheta = 0.2f; // radians
        uint32_t sensorStaleMs = 500; // skip EKF correction if no fresh MCL sensor data for this long
};

struct DebugInfo {
        lemlib::Pose fusedPose {0, 0, 0};
        lemlib::Pose mclPose {0, 0, 0};
        float mclConfidence = 0.0f;
        float mclEss = 0.0f;
        float lastNis = -1.0f;
        float mclVarX = 0.0f;
        float mclVarY = 0.0f;
        float mclVarTheta = 0.0f;
        int activeSensors = 0;
        bool mclValid = false;
        bool sensorsStale = false;
        bool correctionAccepted = false;
        uint32_t correctionRejectMask = 0;
        float measurementPoseDelta = 0.0f;
        float measurementHeadingDelta = 0.0f;
        int correctionInlierSensors = 0;
        float correctionMeanResidual = 0.0f;
        float correctionMaxResidual = 0.0f;
        int candidateStableScans = 0;
        int requiredStableScans = 0;
};

struct TraceSensorInfo {
        float distance = -1.0f;
        float objectVelocity = 0.0f;
        float expectedDistance = -1.0f;
        float residual = 0.0f;
        int confidence = -1;
        int objectSize = -1;
        std::uint16_t readingAgeMs = 0;
        std::uint16_t readingSeq = 0;
        bool inRange = false;
        bool confidenceAccepted = false;
        bool used = false;
        bool readingChanged = false;
};

struct TraceSample {
        uint32_t timeMs = 0;
        uint32_t odomSeq = 0;
        lemlib::OdomDelta odomDelta {};
        lemlib::OdomTelemetry odomTelemetry {};
        lemlib::Pose odomPose {0, 0, 0};
        lemlib::Pose odomOnlyPose {0, 0, 0};
        lemlib::Pose ekfPose {0, 0, 0};
        lemlib::Pose appliedPose {0, 0, 0};
        lemlib::Pose mclPose {0, 0, 0};
        float targetCorrectionX = 0.0f;
        float targetCorrectionY = 0.0f;
        float targetCorrectionTheta = 0.0f;
        float appliedCorrectionX = 0.0f;
        float appliedCorrectionY = 0.0f;
        float appliedCorrectionTheta = 0.0f;
        float mclConfidence = 0.0f;
        float mclEss = 0.0f;
        float lastNis = -1.0f;
        float mclVarX = 0.0f;
        float mclVarY = 0.0f;
        float mclVarTheta = 0.0f;
        int activeSensors = 0;
        bool mclValid = false;
        bool sensorsStale = false;
        bool correctionAccepted = false;
        uint32_t correctionRejectMask = 0;
        float measurementPoseDelta = 0.0f;
        float measurementHeadingDelta = 0.0f;
        int correctionInlierSensors = 0;
        float correctionMeanResidual = 0.0f;
        float correctionMaxResidual = 0.0f;
        int candidateStableScans = 0;
        int requiredStableScans = 0;
        std::array<TraceSensorInfo, 4> sensors {};
};

struct LocalizationConfig {
        FieldConfig field {};
        MCLConfig mcl {};
        EKFConfig ekf {};
        FusionConfig fusion {};
        std::array<SensorConfig, 4> sensors {};
};

void configure(const LocalizationConfig& config);
void start();
void stop();
bool isRunning();
void setMotionCorrectionSuppressed(bool suppressed);
bool isMotionCorrectionSuppressed();
void syncPose(lemlib::Pose pose);
void syncPose(lemlib::Pose pose, uint32_t seq);

lemlib::Pose getFusedPose(bool radians = true);
DebugInfo getDebugInfo(bool radians = true);
LocalizationConfig getConfig();
void setTraceEnabled(bool enabled);
void clearTrace();
TraceSample getLatestTraceSample(bool radians = true);
std::vector<TraceSample> getTraceSamples(bool radians = true);
} // namespace lemlib::localization
