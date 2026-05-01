#pragma once

#include <array>
#include <random>
#include <vector>

#include "lemlib/chassis/odom.hpp"
#include "lemlib/localization/math.hpp"
#include "lemlib/pose.hpp"
#include "pros/distance.hpp"

namespace lemlib::localization {
struct FieldConfig {
        float minX = -72.0f;
        float maxX = 72.0f;
        float minY = -72.0f;
        float maxY = 72.0f;
        float fieldMargin = 0.0f; // inches, shrink valid region by this (robot radius + tolerance)
        float obstacleMargin = 0.0f; // inches, inflate obstacle collision boundaries

        struct Obstacle {
                enum class Type { Circle, Rect };
                Type type = Type::Rect;
                float x = 0.0f;
                float y = 0.0f;
                float radius = 0.0f; // for circles
                float halfW = 0.0f; // for rectangles
                float halfH = 0.0f; // for rectangles
        };

        std::vector<Obstacle> obstacles {};
};

struct SensorConfig {
        pros::Distance* sensor = nullptr;
        float dx = 0.0f; // inches, +right
        float dy = 0.0f; // inches, +forward
        float dtheta = 0.0f; // radians, +clockwise
        float minRange = 1.0f; // inches
        float maxRange = 80.0f; // inches
        int minConfidence = 30; // 0-63, PROS confidence
};

struct SensorMeasurementDebug {
        float distance = -1.0f;
        int confidence = -1;
        bool inRange = false;
        bool confidenceAccepted = false;
        bool used = false;
};

struct SensorObservation {
        float distance = -1.0f; // inches
        int confidence = -1; // -1 means unavailable
        bool available = false;
};

struct MCLConfig { // TUNE
        int numParticles = 500;
        float sensorStd = 2.0f; // inches
        float outlierThreshold = 6.0f; // inches
        float outlierWeight = 0.2f;
        float minWeight = 1e-9f;
        float resampleEssRatio = 0.5f;
        float motionStdXY = 0.35f; // inches
        float motionStdXYPerIn = 0.03f;
        float motionStdTheta = 0.01f; // radians
        float motionStdThetaPerRad = 0.02f;
        float motionLateralStdPerRad = 0.0f; // inches of extra side spread per radian turned
        float motionLateralStdPerInRad = 0.0f; // inches of extra side spread per inch-forward*radian-turn
        float motionLateralBiasPerRad = 0.0f; // inches of signed side drift per radian turned
        float motionLateralBiasPerInRad = 0.0f; // inches of signed side drift per inch-forward*radian-turn
        int minActiveSensors =
            2; // If too high, MCL often refuses to produce valid updates. If too low, weak updates slip through.
        float maxStepXY = 1.0f; // deprecated: fusion-stage correction limits are used instead, no need to tune
        float maxStepTheta = 0.35f; // deprecated: fusion-stage correction limits are used instead, no need to tune
        float initStdXY = 6.0f; // inches initialize particles in a circle around the robot
        float initStdTheta = 0.053f; // radians (~3 deg) initialize particles in a circle around the robot
        float confidenceMaxSpread = 24.0f; // inches (2 ft)
        float rougheningStdXY = 0.15f; // inches, post-resample jitter
        float rougheningStdTheta = 0.005f; // radians, post-resample jitter
        float sideRougheningStdPerRad = 0.0f; // inches of extra side spread per accumulated radian since last scan
};

struct MCLMeasurement {
        lemlib::Pose pose {0, 0, 0};
        Mat3 covariance = Mat3::identity(1.0f);
        float confidence = 0.0f; // 0..1
        float ess = 0.0f;
        int activeSensors = 0;
        bool valid = false;
        std::array<SensorMeasurementDebug, 4> sensorReadings {};
};

class MCL {
    public:
        void configure(const FieldConfig& field, const MCLConfig& cfg, const std::array<SensorConfig, 4>& sensors);
        void reset(const lemlib::Pose& pose);
        void predict(const lemlib::OdomDelta& delta);
        MCLMeasurement update();
        MCLMeasurement update(const std::array<SensorObservation, 4>& observations);
    private:
        struct Particle {
                float x = 0.0f;
                float y = 0.0f;
                float theta = 0.0f;
                float weight = 1.0f;
        };

        FieldConfig field_ {};
        MCLConfig cfg_ {};
        std::array<SensorConfig, 4> sensors_ {};
        std::vector<Particle> particles_;
        // Avoid std::random_device during static initialization on embedded targets.
        std::mt19937 rng_ {1658u};
        bool configured_ = false;
        float accumulatedTurnMag_ = 0.0f;

        float raycastToField(float sx, float sy, float dirX, float dirY) const;
        float expectedDistance(const Particle& p, const SensorConfig& sensor) const;
        bool isInsideObstacle(float x, float y) const;
};
} // namespace lemlib::localization
