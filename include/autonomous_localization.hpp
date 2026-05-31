#pragma once

#include "lemlib/api.hpp"

namespace autonomous_localization {
struct SensorSnapshot {
    float distance = -1.0f;
    int confidence = -1;
};

struct DistanceSnapshot {
    SensorSnapshot front {};
    SensorSnapshot right {};
    SensorSnapshot back {};
    SensorSnapshot left {};
};

struct RelocalizationSummary {
    bool attempted = false;
    bool success = false;
    const char* status = "not_attempted";
    const char* headingMode = "imu_fixed";
    lemlib::Pose pose {0, 0, 0};
    float imuHeading = 0.0f;
    float confidence = 0.0f;
    float varX = 0.0f;
    float varY = 0.0f;
    float varTheta = 0.0f;
    float meanResidual = 0.0f;
    float maxResidual = 0.0f;
    int activeSensors = 0;
    int scoredSensors = 0;
    int iterations = 0;
    int headingHypotheses = 0;
    DistanceSnapshot distances {};
};

class StartRelativeChassis {
public:
    void setOrigin(const lemlib::Pose& absolutePose, bool radians = true);
    lemlib::Pose getOrigin(bool radians = false) const;

    void setPose(float x, float y, float theta, bool radians = false);
    void setPose(lemlib::Pose pose, bool radians = false);
    lemlib::Pose getPose(bool radians = false, bool standardPos = false) const;

    void turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params = {}, bool async = true);
    void turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);
    void swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
                        lemlib::SwingToHeadingParams params = {}, bool async = true);
    void swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
                      lemlib::SwingToPointParams params = {}, bool async = true);
    void moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {},
                    bool async = true);
    void moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params = {}, bool async = true);
    void waitUntil(float dist);
    void waitUntilDone();
    void cancelMotion();
    void cancelAllMotions();

private:
    lemlib::Pose m_originAbsRad {0, 0, 0};
    bool m_hasOrigin = false;
};

void ensureLocalizationRunning();
void applyRelocalizedPose(const lemlib::Pose& pose);
RelocalizationSummary performGlobalRelocalization();
bool beginAutonomousWithRelocalization(RelocalizationSummary* summary = nullptr);
bool beginAutonomousWithFixedStart(const lemlib::Pose& absoluteStartPose, StartRelativeChassis* localChassis = nullptr);
// Relocalize against the walls and anchor the start-relative route frame.
// On a confident solve the local origin becomes the true field pose so
// localization corrects the run; on failure it falls back to the fixed start so
// the robot runs the identical start-relative route on odometry from local
// (0, 0, 0). Returns true if relocalization succeeded (the route runs either way).
bool beginAutonomousWithRelocalizationOrFixedStart(const lemlib::Pose& fixedStartAbsolute,
                                                   StartRelativeChassis* localChassis = nullptr,
                                                   RelocalizationSummary* summary = nullptr);
} // namespace autonomous_localization
