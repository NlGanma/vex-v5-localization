#include "localization_config.hpp"

#include "robot.hpp"

#include <array>
#include <algorithm>

namespace {
constexpr float kFieldWidth = 140.43f;
constexpr float kFieldHeight = 140.41f;

struct AbsRect {
    float xMin;
    float xMax;
    float yMin;
    float yMax;
};
} // namespace

lemlib::localization::LocalizationConfig buildLocalizationConfig() {
    lemlib::localization::LocalizationConfig loc {};

    // Field bounds from the 2025-2026 V5RC Push Back field reference sheet (inches).
    const float halfW = kFieldWidth * 0.5f;
    const float halfH = kFieldHeight * 0.5f;
    loc.field.minX = -halfW;
    loc.field.maxX = halfW;
    loc.field.minY = -halfH;
    loc.field.maxY = halfH;
    loc.field.fieldMargin = 10.0f; // robot radius (~9") + 1" construction tolerance
    loc.field.obstacleMargin = 1.0f; // requested 1" tolerance via runtime obstacle inflation
    loc.field.obstacles.reserve(11);

    auto cx = [halfW](float xAbs) { return xAbs - halfW; };
    auto cy = [halfH](float yAbs) { return yAbs - halfH; };
    auto makeRectAbs = [&](float xMin, float xMax, float yMin, float yMax) {
        lemlib::localization::FieldConfig::Obstacle obstacle {};
        obstacle.type = lemlib::localization::FieldConfig::Obstacle::Type::Rect;
        obstacle.x = cx((xMin + xMax) * 0.5f);
        obstacle.y = cy((yMin + yMax) * 0.5f);
        obstacle.halfW = (xMax - xMin) * 0.5f;
        obstacle.halfH = (yMax - yMin) * 0.5f;
        return obstacle;
    };
    auto makeCircleAbs = [&](float xCenter, float yCenter, float radius) {
        lemlib::localization::FieldConfig::Obstacle obstacle {};
        obstacle.type = lemlib::localization::FieldConfig::Obstacle::Type::Circle;
        obstacle.x = cx(xCenter);
        obstacle.y = cy(yCenter);
        obstacle.radius = radius;
        return obstacle;
    };

    // Model the XY union of all geometry occupying z in [5", 7"] above the field tiles.
    // Source sheets:
    // - Appendix A-5: Field Reference Specifications - V5RC (field placements)
    // - Appendix A-16: Long Goal Specifications
    // - Appendix A-17: Center Goal Specifications
    // - Appendix A-19: Loader Specifications
    //
    // In this band:
    // - field walls come from the min/max bounds above
    // - loader bodies occupy the band, so their map footprint is the field-reference footprint
    // - long goals intersect only at their 5.87" support stands, not the tray body
    // - center goals intersect only at the central support cross; the upper/lower tray bodies begin above this band

    constexpr AbsRect kLeftLoader {0.0f, 24.69f, 60.77f, 79.64f};
    loc.field.obstacles.push_back(makeRectAbs(kLeftLoader.xMin, kLeftLoader.xMax, kLeftLoader.yMin, kLeftLoader.yMax));
    loc.field.obstacles.push_back(
        makeRectAbs(kFieldWidth - kLeftLoader.xMax, kFieldWidth, kLeftLoader.yMin, kLeftLoader.yMax));

    constexpr float kLongGoalSupportRadius = 5.87f * 0.5f;
    const auto addLongGoalSupports = [&](const AbsRect& footprint) {
        const float yCenter = (footprint.yMin + footprint.yMax) * 0.5f;
        loc.field.obstacles.push_back(
            makeCircleAbs(footprint.xMin + kLongGoalSupportRadius, yCenter, kLongGoalSupportRadius));
        loc.field.obstacles.push_back(
            makeCircleAbs(footprint.xMax - kLongGoalSupportRadius, yCenter, kLongGoalSupportRadius));
    };
    const std::array<AbsRect, 2> longGoalFootprints {{
        {45.83f, 94.62f, 46.63f, 49.88f},
        {45.83f, 94.62f, 90.53f, 93.78f},
    }};
    for (const auto& footprint : longGoalFootprints) addLongGoalSupports(footprint);

    const std::array<AbsRect, 5> centerGoalSupportRects {{
        {68.61f, 71.84f, 68.59f, 71.82f}, // center square
        {68.61f, 71.84f, 71.82f, 75.05f}, // north arm
        {68.61f, 71.84f, 65.36f, 68.59f}, // south arm
        {65.38f, 68.61f, 68.59f, 71.82f}, // west arm
        {71.84f, 75.07f, 68.59f, 71.82f}, // east arm
    }};
    for (const auto& rect : centerGoalSupportRects) {
        loc.field.obstacles.push_back(makeRectAbs(rect.xMin, rect.xMax, rect.yMin, rect.yMax));
    }

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
    loc.fusion.maxCorrectionXY = 0.08f;
    loc.fusion.maxCorrectionTheta = lemlib::degToRad(0.5f);
    loc.fusion.initStdXY = 2.5f;
    loc.fusion.initStdTheta = lemlib::degToRad(6.0f);
    loc.fusion.sensorStaleMs = 300;

    // Distance sensors mounted in robot coordinates: dx +right, dy +forward,
    // dtheta clockwise from the robot front.
    loc.sensors = std::array<lemlib::localization::SensorConfig, 4> {{
        {&distFront, -6.5625f, 0.5f, 0.0f, 5.0f, 96.0f, 15},
        {&distRight, 7.75f, -2.0f, lemlib::degToRad(91.0f), 5.0f, 96.0f, 10},
        {&distBack, 3.375f, -9.5f, lemlib::degToRad(183.0f), 5.0f, 96.0f, 10},
        {&distLeft, -4.75f, -3.875f, lemlib::degToRad(-92.0f), 5.0f, 96.0f, 15},
    }};

    return loc;
}
