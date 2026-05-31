#pragma once

#include <cstdint>
#include <vector>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"

namespace lemlib {
/**
 * @brief Local odometry delta computed each update
 *
 * localX is lateral (positive left), localY is forward.
 * theta is in radians, dt in seconds.
 */
struct OdomDelta {
        float localX = 0;
        float localY = 0;
        float deltaTheta = 0;
        float dt = 0.01f;
        uint32_t seq = 0;
};

struct OdomSnapshot {
        Pose pose {0, 0, 0};
        uint32_t seq = 0;
};

struct OdomTelemetry {
        float dt = 0.01f;
        float headingBefore = 0.0f;
        float headingAfter = 0.0f;
        float deltaHeading = 0.0f;
        float vertical1Raw = 0.0f;
        float vertical2Raw = 0.0f;
        float horizontal1Raw = 0.0f;
        float horizontal2Raw = 0.0f;
        float imuRaw = 0.0f;
        float deltaVertical1 = 0.0f;
        float deltaVertical2 = 0.0f;
        float deltaHorizontal1 = 0.0f;
        float deltaHorizontal2 = 0.0f;
        float deltaImu = 0.0f;
        float selectedVerticalRaw = 0.0f;
        float selectedHorizontalRaw = 0.0f;
        float selectedDeltaVertical = 0.0f;
        float selectedDeltaHorizontal = 0.0f;
        float selectedVerticalOffset = 0.0f;
        float selectedHorizontalOffset = 0.0f;
        bool usedHorizontalHeadingPair = false;
        bool usedVerticalHeadingPair = false;
        bool usedImuHeading = false;
        bool usedHeadingFallback = false;
        uint32_t seq = 0;
};

/**
 * @brief Set the sensors to be used for odometry
 *
 * @param sensors the sensors to be used
 * @param drivetrain drivetrain to be used
 */
void setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain);
/**
 * @brief Get the pose of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return Pose
 */
Pose getPose(bool radians = false);
/**
 * @brief Set the Pose of the robot
 *
 * @param pose the new pose
 * @param radians true if theta is in radians, false if in degrees. False by default
 */
void setPose(Pose pose, bool radians = false);
/**
 * @brief Get the most recent local odometry delta
 *
 * @return OdomDelta
 */
OdomDelta getOdomDelta();
/**
 * @brief Get all odometry deltas newer than the provided sequence number
 *
 * This is primarily used internally by the localization task so it can replay
 * every odom increment instead of silently skipping updates under scheduler lag.
 *
 * @param seq the last sequence number already consumed
 * @return std::vector<OdomDelta>
 */
std::vector<OdomDelta> getOdomDeltasSince(uint32_t seq);
/**
 * @brief Get a pose/sequence snapshot for localization synchronization
 *
 * The returned sequence corresponds to the pose already reflected in the
 * odometry state, so localization can reset and resume replay without
 * duplicating or skipping a delta.
 *
 * @return OdomSnapshot
 */
OdomSnapshot getOdomSnapshot();
/**
 * @brief Get the most recent raw odometry telemetry
 *
 * This is primarily used for offline localization tuning. The readings are the
 * raw wheel / IMU values that produced the last odometry increment.
 *
 * @return OdomTelemetry
 */
OdomTelemetry getOdomTelemetry();
/**
 * @brief Get odometry telemetry for a specific sequence number
 *
 * @param seq the sequence number to look up
 * @return OdomTelemetry
 */
OdomTelemetry getOdomTelemetryForSeq(uint32_t seq);
/**
 * @brief Get the speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return lemlib::Pose
 */
Pose getSpeed(bool radians = false);
/**
 * @brief Get the local speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return lemlib::Pose
 */
Pose getLocalSpeed(bool radians = false);
/**
 * @brief Estimate the pose of the robot after a certain amount of time
 *
 * @param time time in seconds
 * @param radians False for degrees, true for radians. False by default
 * @return lemlib::Pose
 */
Pose estimatePose(float time, bool radians = false);
/**
 * @brief Update the pose of the robot
 *
 */
void update();
/**
 * @brief Initialize the odometry system
 *
 */
void init();

namespace detail {
/**
 * @brief Internal pose setter used by the localization task.
 *
 * This updates odom state without resynchronizing the localization filters.
 */
void setPoseSilent(Pose pose, bool radians = false);
/**
 * @brief Seq-guarded silent pose setter used by the localization task.
 *
 * Writes the corrected pose only if the published odom sequence still equals
 * expectedSeq, holding the odom update lock (same order as the tracking task)
 * so a concurrent odom integration step cannot be silently overwritten.
 *
 * @return true if the write was applied, false if odom advanced (write skipped)
 */
bool setPoseSilentIfSeq(Pose pose, uint32_t expectedSeq, bool radians = false);
} // namespace detail
} // namespace lemlib
