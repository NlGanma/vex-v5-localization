#include "localization_config.hpp"

#include "robot.hpp"

#include <array>

namespace {
constexpr float kFieldWidth = 140.43f;
constexpr float kFieldHeight = 140.41f;
} // namespace

lemlib::localization::LocalizationConfig buildLocalizationConfig() {
    lemlib::localization::LocalizationConfig loc {};

    // VEX field perimeter in inches, using field-centered coordinates.
    const float halfW = kFieldWidth * 0.5f;
    const float halfH = kFieldHeight * 0.5f;
    loc.field.minX = -halfW;
    loc.field.maxX = halfW;
    loc.field.minY = -halfH;
    loc.field.maxY = halfH;
    loc.field.fieldMargin = 10.0f; // robot radius (~9") + 1" construction tolerance
    loc.field.obstacleMargin = 0.0f;
    loc.field.obstacles.reserve(1);

    lemlib::localization::FieldConfig::Obstacle centerObstacle {};
    centerObstacle.type = lemlib::localization::FieldConfig::Obstacle::Type::Rect;
    centerObstacle.x = 0.0f;
    centerObstacle.y = 0.0f;
    centerObstacle.halfW = 3.0f;
    centerObstacle.halfH = 3.0f;
    loc.field.obstacles.push_back(centerObstacle);

    // Conservative localization defaults for an 18" VEX robot with four distance sensors and low lateral drift.
    loc.mcl.numParticles = 450;
    loc.mcl.sensorStd = 3.0f;
    loc.mcl.outlierThreshold = 7.0f;
    loc.mcl.outlierWeight = 0.22f;
    loc.mcl.minActiveSensors = 2;
    loc.mcl.motionStdXY = 0.20f;
    loc.mcl.motionStdXYPerIn = 0.018f;
    loc.mcl.motionStdTheta = 0.008f;
    loc.mcl.motionStdThetaPerRad = 0.018f;
    loc.mcl.motionLateralStdPerRad = 0.08f;
    loc.mcl.motionLateralStdPerInRad = 0.0015f;
    loc.mcl.motionLateralBiasPerRad = 0.0f;
    loc.mcl.motionLateralBiasPerInRad = 0.0f;
    loc.mcl.initStdXY = 2.0f;
    loc.mcl.initStdTheta = lemlib::degToRad(2.0f);
    loc.mcl.confidenceMaxSpread = 32.0f;
    loc.mcl.rougheningStdXY = 0.08f;
    loc.mcl.rougheningStdTheta = 0.0035f;
    loc.mcl.sideRougheningStdPerRad = 0.025f;

    loc.ekf.processStdXY = 0.08f;
    loc.ekf.processStdXYPerIn = 0.025f;
    loc.ekf.processStdTheta = 0.012f;
    loc.ekf.processStdThetaPerRad = 0.025f;
    loc.ekf.processStdLateralPerRad = 0.03f;
    loc.ekf.processStdLateralPerInRad = 0.0015f;

    loc.fusion.nisGate = 8.0f;
    loc.fusion.minCorrectionSensors = 3;
    loc.fusion.minConfidence = 0.55f;
    loc.fusion.maxVarXY = 81.0f;
    loc.fusion.maxVarTheta = 0.10f;
    loc.fusion.maxMeasurementDeltaXY = 2.5f;
    loc.fusion.maxMeasurementDeltaTheta = lemlib::degToRad(4.0f);
    loc.fusion.maxCorrectionXY = 0.015f;
    loc.fusion.maxCorrectionTheta = lemlib::degToRad(0.5f);
    loc.fusion.initStdXY = 2.5f;
    loc.fusion.initStdTheta = lemlib::degToRad(6.0f);
    loc.fusion.sensorStaleMs = 300;

    // Distance sensors mounted in robot coordinates: dx +right, dy +forward,
    // dtheta clockwise from the robot front.
    loc.sensors = std::array<lemlib::localization::SensorConfig, 4> {{
        {&distFront, -2.749f, -1.875f, lemlib::degToRad(-33.95f), 5.0f, 96.0f, 15},
        {&distRight, 9.938f, -1.25f, lemlib::degToRad(100.75f), 5.0f, 96.0f, 10},
        {&distBack, -0.625f, -10.749f, lemlib::degToRad(202.10f), 5.0f, 96.0f, 10},
        {&distLeft, -6.437f, -2.374f, lemlib::degToRad(-115.40f), 5.0f, 96.0f, 15},
    }};

    return loc;
}
