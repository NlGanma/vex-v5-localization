#include "localization_tune.hpp"

#include "main.h"
#include "lemlib/logger/stdout.hpp"
#include "pros/apix.h"
#include "robot_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <fcntl.h>
#include <string>
#include <string_view>
#include <unistd.h>
#include <vector>

namespace localization_tune {
namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr std::uint32_t kTuneSettleMs = 5;
constexpr std::uint32_t kTuneInitialSettleMs = 15;
constexpr std::uint32_t kTuneMotionPollMs = 5;
constexpr std::uint32_t kTuneMotionCancelSettleMs = 5;
constexpr std::uint32_t kTunePostMoveCorrectionWindowMs = 120;
constexpr float kTunePostMoveGoodNis = 2.0f;
constexpr std::uint32_t kTuneRejectMotionSuppressedMask = 1u << 9;
constexpr int kTunePostMoveMinInlierSensors = 2;
constexpr float kTunePostMoveMaxResidual = 4.0f;
constexpr int kTuneCheckpointCapacity = 32;
constexpr const char* kTuneLogPath = "/usd/localization_tune_latest.txt";
constexpr const char* kTuneInternalLogPath = "localization_tune_latest_internal.txt";
constexpr int kTuneExportTapMinX = 200;
constexpr int kTuneExportTapMinY = 120;
constexpr std::uint32_t kTuneExportFeedbackMs = 2000;
constexpr std::uintptr_t kStdoutStreamId = 0x74756f73u; // little-endian "sout"
constexpr std::array<const char*, 4> kTraceSensorNames {{"front", "right", "back", "left"}};

struct TuneCheckpoint {
    const char* name = "Not recorded";
    lemlib::Pose expected {0, 0, 0};
    lemlib::Pose reported {0, 0, 0};
    lemlib::localization::DebugInfo debug {};
    DistanceSnapshot distances {};
    std::uint32_t odomSeq = 0;
    bool recorded = false;
};

enum class TuneMotionKind { Move, Turn, Drive };

struct AutonomousRouteStep {
    TuneMotionKind kind = TuneMotionKind::Move;
    const char* name = "Unnamed";
    float localRight = 0.0f;
    float localForward = 0.0f;
    float headingOffsetDeg = 0.0f;
    int timeoutMs = 2000;
    bool forwards = true;
    float lead = 0.6f;
    float maxSpeed = 100.0f;
    float minSpeed = 0.0f;
    float earlyExitRange = 0.0f;
};

enum class TuneRunMode { Idle, AutonomousRelocalized };
enum class TuneTestCase {
    NormalRoute = 0,
    TurnCenter = 1,
    StraightScale = 2,
    SquareLoop = 3,
    DriveProbe = 4,
    SensorAngle = 5,
    SquareCross = 6
};
enum class TuneLogOutputMode { Lean, DeepDive };

struct TuneTestDefinition {
    TuneTestCase id = TuneTestCase::TurnCenter;
    const char* name = "Turn center";
    const char* objective = "Angular PID and rotational center";
    const AutonomousRouteStep* steps = nullptr;
    size_t stepCount = 0;
};

constexpr TuneLogOutputMode kTuneLogOutputMode = TuneLogOutputMode::DeepDive;
constexpr TuneTestCase kDefaultTuneTestCase = TuneTestCase::TurnCenter;
constexpr std::array<AutonomousRouteStep, 7> kTurnCenterRoute {{
    {TuneMotionKind::Turn, "Turn 90", 0.0f, 0.0f, 90.0f, 1400, true, 0.0f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Turn 180", 0.0f, 0.0f, 180.0f, 1600, true, 0.0f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Turn -90", 0.0f, 0.0f, -90.0f, 1600, true, 0.0f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Turn 0", 0.0f, 0.0f, 0.0f, 1400, true, 0.0f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Turn 45", 0.0f, 0.0f, 45.0f, 1200, true, 0.0f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Turn -45", 0.0f, 0.0f, -45.0f, 1400, true, 0.0f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Return 0", 0.0f, 0.0f, 0.0f, 1200, true, 0.0f, 85.0f, 0.0f, 0.0f},
}};
constexpr std::array<AutonomousRouteStep, 8> kSensorAngleRoute {{
    {TuneMotionKind::Turn, "Angle 45", 0.0f, 0.0f, 45.0f, 950, true, 0.0f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Angle 90", 0.0f, 0.0f, 90.0f, 950, true, 0.0f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Angle 135", 0.0f, 0.0f, 135.0f, 1000, true, 0.0f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Angle 180", 0.0f, 0.0f, 180.0f, 1050, true, 0.0f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Angle -135", 0.0f, 0.0f, -135.0f, 1000, true, 0.0f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Angle -90", 0.0f, 0.0f, -90.0f, 950, true, 0.0f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Angle -45", 0.0f, 0.0f, -45.0f, 950, true, 0.0f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Angle 0", 0.0f, 0.0f, 0.0f, 950, true, 0.0f, 85.0f, 0.0f, 0.0f},
}};
constexpr std::array<AutonomousRouteStep, 4> kStraightScaleRoute {{
    {TuneMotionKind::Move, "Forward 24", 0.0f, 24.0f, 0.0f, 2200, true, 0.35f, 85.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Forward 48", 0.0f, 48.0f, 0.0f, 2800, true, 0.35f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Reverse 24", 0.0f, 24.0f, 0.0f, 2600, false, 0.35f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Reverse 0", 0.0f, 0.0f, 0.0f, 2600, false, 0.35f, 80.0f, 0.0f, 0.0f},
}};
// Single 24" square loop that returns home. This fits the ~15s autonomous
// window and is useful as a compact moving-fusion validation route.
constexpr std::array<AutonomousRouteStep, 7> kSquareLoopRoute {{
    {TuneMotionKind::Move, "Square north", 0.0f, 24.0f, 0.0f, 2000, true, 0.42f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face east", 0.0f, 24.0f, 90.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Square east", 24.0f, 24.0f, 90.0f, 2000, true, 0.42f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face south", 24.0f, 24.0f, 180.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Square south", 24.0f, 0.0f, 180.0f, 2000, true, 0.42f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face west", 24.0f, 0.0f, -90.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Return home", 0.0f, 0.0f, -90.0f, 2000, true, 0.42f, 95.0f, 0.0f, 0.0f},
}};
// Longer route for validation outside the 15s match budget. It runs the square
// loop, then adds diagonal cross segments so localization sees oblique motion,
// cardinal turns, diagonal travel, and a final return-home drift check.
constexpr std::array<AutonomousRouteStep, 18> kSquareCrossRoute {{
    {TuneMotionKind::Move, "Square north", 0.0f, 24.0f, 0.0f, 2000, true, 0.42f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face east", 0.0f, 24.0f, 90.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Square east", 24.0f, 24.0f, 90.0f, 2000, true, 0.42f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face south", 24.0f, 24.0f, 180.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Square south", 24.0f, 0.0f, 180.0f, 2000, true, 0.42f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face west", 24.0f, 0.0f, -90.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Return home", 0.0f, 0.0f, -90.0f, 2000, true, 0.42f, 95.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face diagonal NE", 0.0f, 0.0f, 45.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Cross SW to NE", 24.0f, 24.0f, 45.0f, 2600, true, 0.42f, 90.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face diagonal SW", 24.0f, 24.0f, -135.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Cross NE to SW", 0.0f, 0.0f, -135.0f, 2600, true, 0.42f, 90.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face north", 0.0f, 0.0f, 0.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Move to NW", 0.0f, 24.0f, 0.0f, 2000, true, 0.42f, 90.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face diagonal SE", 0.0f, 24.0f, 135.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Cross NW to SE", 24.0f, 0.0f, 135.0f, 2600, true, 0.42f, 90.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Face west final", 24.0f, 0.0f, -90.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
    {TuneMotionKind::Move, "Final return home", 0.0f, 0.0f, -90.0f, 2200, true, 0.42f, 90.0f, 0.0f, 0.0f},
    {TuneMotionKind::Turn, "Final face north", 0.0f, 0.0f, 0.0f, 1200, true, 0.0f, 80.0f, 0.0f, 0.0f},
}};
// Open-loop straight-drive probe (no PID). For Drive steps: maxSpeed = raw tank
// power, timeoutMs = drive duration, forwards = direction. Equal power is sent to
// both sides with the drive curve disabled, so the heading trace reveals drive
// balance: flat heading at equal power => drivetrain balanced (moveToPose's
// controller is at fault); a veer whose sign flips between forward/reverse =>
// drivetrain imbalance. Runs out-and-back at two power levels to stay near start.
constexpr std::array<AutonomousRouteStep, 4> kDriveProbeRoute {{
    {TuneMotionKind::Drive, "Open fwd 60", 0.0f, 18.0f, 0.0f, 1000, true, 0.0f, 60.0f, 0.0f, 0.0f},
    {TuneMotionKind::Drive, "Open rev 60", 0.0f, 0.0f, 0.0f, 1000, false, 0.0f, 60.0f, 0.0f, 0.0f},
    {TuneMotionKind::Drive, "Open fwd 90", 0.0f, 27.0f, 0.0f, 1000, true, 0.0f, 90.0f, 0.0f, 0.0f},
    {TuneMotionKind::Drive, "Open rev 90", 0.0f, 0.0f, 0.0f, 1000, false, 0.0f, 90.0f, 0.0f, 0.0f},
}};

pros::Mutex tuneMutex;
std::array<TuneCheckpoint, kTuneCheckpointCapacity> tuneCheckpoints {};
int tuneCheckpointCount = 0;
int tuneSelectedCheckpoint = 0;
int tuneDisplayPage = 0;
lemlib::Pose tuneStartPose {0, 0, 0};
const char* tuneStatus = "Idle";
const char* tuneStepName = "Autonomous runs relocalized route";
bool tuneTestRunning = false;
TuneRunMode tuneRunMode = TuneRunMode::Idle;
TuneTestCase tuneTestCase = kDefaultTuneTestCase;
RelocalizationSummary relocalizationSummary {};
bool tuneOverlayActive = false;
pros::Task* screenTask = nullptr;
pros::Task* overlayControlTask = nullptr;
pros::Task* terminalReplayTask = nullptr;
pros::Task* manualDriveFallbackTask = nullptr;
pros::Mutex pendingTuneLogMutex;
std::string pendingTuneTerminalLog;
bool pendingTuneTerminalReplay = false;
bool pendingTuneTerminalStreaming = false;
bool pendingTuneTerminalDumpRequested = false;
volatile bool driverControlLoopActive = false;
volatile bool driverDriveLoopTicking = false;
volatile bool manualDriveFallbackActive = false;
const char* tuneExportFeedback = nullptr;
std::uint32_t tuneExportFeedbackUntilMs = 0;
volatile std::uint32_t tuneExportTapSequence = 0;
volatile std::uint32_t tuneExportTapLastMs = 0;

void overlayControlTaskFn();
void manualDriveFallbackTaskFn();
void screenTaskFn();
void terminalReplayTaskFn();
void cachePendingTuneTerminalLog(const std::string& fullLog);

void setTuneExportFeedback(const char* message, std::uint32_t durationMs = kTuneExportFeedbackMs) {
    tuneMutex.take();
    tuneExportFeedback = message;
    tuneExportFeedbackUntilMs = durationMs == 0 ? 0 : pros::millis() + durationMs;
    tuneMutex.give();
}

const char* currentTuneExportFeedback() {
    const char* message = nullptr;
    tuneMutex.take();
    if (tuneExportFeedback != nullptr) {
        const bool expired = tuneExportFeedbackUntilMs != 0 && pros::millis() >= tuneExportFeedbackUntilMs;
        if (expired) {
            tuneExportFeedback = nullptr;
            tuneExportFeedbackUntilMs = 0;
        } else {
            message = tuneExportFeedback;
        }
    }
    tuneMutex.give();
    return message;
}

void handleTuneExportTap() {
    const auto touch = pros::screen::touch_status();
    if (touch.x < kTuneExportTapMinX || touch.y < kTuneExportTapMinY) return;

    const std::uint32_t now = pros::millis();
    if (now - tuneExportTapLastMs < 200) return;

    tuneExportTapLastMs = now;
    ++tuneExportTapSequence;
}

void appendFormat(std::string& out, const char* format, ...) {
    va_list args;
    va_start(args, format);
    va_list copy;
    va_copy(copy, args);
    const int written = std::vsnprintf(nullptr, 0, format, copy);
    va_end(copy);
    if (written <= 0) {
        va_end(args);
        return;
    }

    std::vector<char> buffer(static_cast<size_t>(written) + 1);
    std::vsnprintf(buffer.data(), buffer.size(), format, args);
    va_end(args);
    out.append(buffer.data(), static_cast<size_t>(written));
}

SensorSnapshot captureSensor(pros::Distance& sensor) {
    SensorSnapshot snapshot {};
    const std::int32_t distanceMm = sensor.get_distance();
    if (distanceMm <= 0 || distanceMm >= 9000) return snapshot;

    snapshot.distance = static_cast<float>(distanceMm) / 25.4f;
    if (distanceMm >= 200) snapshot.confidence = sensor.get_confidence();
    return snapshot;
}

DistanceSnapshot captureDistances() {
    return DistanceSnapshot {
        captureSensor(distFront),
        captureSensor(distRight),
        captureSensor(distBack),
        captureSensor(distLeft),
    };
}

DistanceSnapshot captureDistances(const lemlib::localization::TraceSample& sample) {
    auto convert = [](const lemlib::localization::TraceSensorInfo& sensor) {
        SensorSnapshot snapshot {};
        snapshot.distance = sensor.distance;
        snapshot.confidence = sensor.confidence;
        return snapshot;
    };
    return DistanceSnapshot {
        convert(sample.sensors[0]),
        convert(sample.sensors[1]),
        convert(sample.sensors[2]),
        convert(sample.sensors[3]),
    };
}

float wrapDegrees(float degrees) {
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees <= -180.0f) degrees += 360.0f;
    return degrees;
}

float wrapRadians(float radians) {
    while (radians > kPi) radians -= 2.0f * kPi;
    while (radians <= -kPi) radians += 2.0f * kPi;
    return radians;
}

lemlib::Pose offsetPose(const lemlib::Pose& start, float localRight, float localForward, float deltaTheta) {
    const float sinTheta = std::sin(start.theta);
    const float cosTheta = std::cos(start.theta);
    return lemlib::Pose(start.x + localRight * cosTheta + localForward * sinTheta,
                        start.y - localRight * sinTheta + localForward * cosTheta,
                        wrapRadians(start.theta + deltaTheta));
}

lemlib::Pose autonomousRouteTarget(const lemlib::Pose& start, const AutonomousRouteStep& step) {
    return offsetPose(start, step.localRight, step.localForward, lemlib::degToRad(step.headingOffsetDeg));
}

void clearTuneResults() {
    tuneMutex.take();
    tuneCheckpoints = {};
    tuneCheckpointCount = 0;
    tuneSelectedCheckpoint = 0;
    tuneDisplayPage = 0;
    tuneMutex.give();
}

void clearRelocalizationSummary() {
    tuneMutex.take();
    relocalizationSummary = {};
    tuneMutex.give();
}

const char* tuneRunModeName(TuneRunMode mode) {
    switch (mode) {
        case TuneRunMode::AutonomousRelocalized: return "autonomous_relocalized";
        case TuneRunMode::Idle:
        default: return "idle";
    }
}

TuneTestCase tuneTestCaseFromNumber(int testNumber) {
    switch (testNumber) {
        case 2: return TuneTestCase::StraightScale;
        case 3: return TuneTestCase::SquareLoop;
        case 4: return TuneTestCase::DriveProbe;
        case 5: return TuneTestCase::SensorAngle;
        case 6: return TuneTestCase::SquareCross;
        case 1:
        default: return TuneTestCase::TurnCenter;
    }
}

int tuneTestCaseNumber(TuneTestCase testCase) {
    switch (testCase) {
        case TuneTestCase::NormalRoute: return 0;
        case TuneTestCase::StraightScale: return 2;
        case TuneTestCase::SquareLoop: return 3;
        case TuneTestCase::DriveProbe: return 4;
        case TuneTestCase::SensorAngle: return 5;
        case TuneTestCase::SquareCross: return 6;
        case TuneTestCase::TurnCenter:
        default: return 1;
    }
}

const char* tuneMotionKindName(TuneMotionKind kind) {
    switch (kind) {
        case TuneMotionKind::Turn: return "turn";
        case TuneMotionKind::Drive: return "drive";
        case TuneMotionKind::Move:
        default: return "move";
    }
}

TuneTestDefinition tuneTestDefinition(TuneTestCase testCase) {
    switch (testCase) {
        case TuneTestCase::NormalRoute:
            return {testCase, "Normal route", "4-motor validation autonomous with start relocalization/fallback",
                    nullptr, 0};
        case TuneTestCase::StraightScale:
            return {testCase, "Straight scale", "Forward/reverse distance scale and lateral drift",
                    kStraightScaleRoute.data(), kStraightScaleRoute.size()};
        case TuneTestCase::SquareLoop:
            return {testCase, "Square loop", "Square loop returning home: turns + translation + return-home drift vs odom",
                    kSquareLoopRoute.data(), kSquareLoopRoute.size()};
        case TuneTestCase::SquareCross:
            return {testCase, "Square + cross",
                    "Square loop plus diagonal cross segments for oblique moving-fusion validation",
                    kSquareCrossRoute.data(), kSquareCrossRoute.size()};
        case TuneTestCase::DriveProbe:
            return {testCase, "Drive probe", "Open-loop straight drive: drivetrain balance vs moveToPose",
                    kDriveProbeRoute.data(), kDriveProbeRoute.size()};
        case TuneTestCase::SensorAngle:
            return {testCase, "Sensor angle", "Stationary heading sweep for distance-sensor mounting angle fit",
                    kSensorAngleRoute.data(), kSensorAngleRoute.size()};
        case TuneTestCase::TurnCenter:
        default:
            return {TuneTestCase::TurnCenter, "Turn center", "Angular PID and rotational center drift",
                    kTurnCenterRoute.data(), kTurnCenterRoute.size()};
    }
}

const char* tuneLogOutputModeName(TuneLogOutputMode mode) {
    switch (mode) {
        case TuneLogOutputMode::DeepDive: return "deep_dive";
        case TuneLogOutputMode::Lean:
        default: return "lean";
    }
}

void appendLocalizationConfig(std::string& out) {
    const auto config = lemlib::localization::getConfig();
    appendFormat(out, "field_min=%.2f %.2f\n", config.field.minX, config.field.minY);
    appendFormat(out, "field_max=%.2f %.2f\n", config.field.maxX, config.field.maxY);
    appendFormat(out, "field_margin=%.2f obstacle_margin=%.2f obstacle_count=%lu\n", config.field.fieldMargin,
                 config.field.obstacleMargin, static_cast<unsigned long>(config.field.obstacles.size()));
    appendFormat(out, "fusion ekf_ms=%lu mcl_ms=%lu nis=%.2f min_sensors=%d min_conf=%.2f stale_ms=%lu blend=%.2f\n",
                 static_cast<unsigned long>(config.fusion.ekfPeriodMs),
                 static_cast<unsigned long>(config.fusion.mclPeriodMs), config.fusion.nisGate,
                 config.fusion.minCorrectionSensors, config.fusion.minConfidence,
                 static_cast<unsigned long>(config.fusion.sensorStaleMs), config.fusion.blend);
    appendFormat(out, "fusion max_var_xy=%.3f max_var_th=%.5f max_meas_xy=%.3f max_meas_th=%.3f max_corr_xy=%.3f max_corr_th=%.5f\n",
                 config.fusion.maxVarXY, config.fusion.maxVarTheta, config.fusion.maxMeasurementDeltaXY,
                 lemlib::radToDeg(config.fusion.maxMeasurementDeltaTheta), config.fusion.maxCorrectionXY,
                 config.fusion.maxCorrectionTheta);
    appendFormat(out, "mcl particles=%d sensor_std=%.3f outlier_th=%.3f outlier_weight=%.3f min_active=%d\n",
                 config.mcl.numParticles, config.mcl.sensorStd, config.mcl.outlierThreshold,
                 config.mcl.outlierWeight, config.mcl.minActiveSensors);
    appendFormat(out, "mcl motion_xy=%.4f motion_xy_per_in=%.4f motion_th=%.5f motion_th_per_rad=%.5f\n",
                 config.mcl.motionStdXY, config.mcl.motionStdXYPerIn, config.mcl.motionStdTheta,
                 config.mcl.motionStdThetaPerRad);
    appendFormat(out,
                 "mcl lat_std_per_rad=%.4f lat_std_per_in_rad=%.4f lat_bias_per_rad=%.4f lat_bias_per_in_rad=%.4f\n",
                 config.mcl.motionLateralStdPerRad, config.mcl.motionLateralStdPerInRad,
                 config.mcl.motionLateralBiasPerRad, config.mcl.motionLateralBiasPerInRad);
    appendFormat(out,
                 "mcl init_xy=%.3f init_th=%.5f conf_spread=%.3f rough_xy=%.4f rough_th=%.5f side_rough_per_rad=%.4f\n",
                 config.mcl.initStdXY, config.mcl.initStdTheta, config.mcl.confidenceMaxSpread,
                 config.mcl.rougheningStdXY, config.mcl.rougheningStdTheta, config.mcl.sideRougheningStdPerRad);
    appendFormat(out,
                 "ekf proc_xy=%.4f proc_xy_per_in=%.4f proc_th=%.5f proc_th_per_rad=%.5f lat_per_rad=%.4f lat_per_in_rad=%.4f\n",
                 config.ekf.processStdXY, config.ekf.processStdXYPerIn, config.ekf.processStdTheta,
                 config.ekf.processStdThetaPerRad, config.ekf.processStdLateralPerRad,
                 config.ekf.processStdLateralPerInRad);
    for (size_t i = 0; i < config.sensors.size(); ++i) {
        const auto& sensor = config.sensors[i];
        appendFormat(out,
                     "sensor%lu dx=%.3f dy=%.3f dtheta_deg=%.2f min=%.2f max=%.2f min_conf=%d configured=%d\n",
                     static_cast<unsigned long>(i), sensor.dx, sensor.dy, lemlib::radToDeg(sensor.dtheta),
                     sensor.minRange, sensor.maxRange, sensor.minConfidence, sensor.sensor != nullptr);
    }
}

std::string buildTuneReport(TuneLogOutputMode outputMode = kTuneLogOutputMode) {
    std::array<TuneCheckpoint, kTuneCheckpointCapacity> checkpoints {};
    int checkpointCount = 0;
    const char* status = nullptr;
    const char* stepName = nullptr;
    lemlib::Pose startPoseSnapshot {0, 0, 0};
    TuneRunMode mode = TuneRunMode::Idle;
    TuneTestCase testCase = kDefaultTuneTestCase;
    RelocalizationSummary relocalize {};

    tuneMutex.take();
    checkpoints = tuneCheckpoints;
    checkpointCount = tuneCheckpointCount;
    status = tuneStatus;
    stepName = tuneStepName;
    startPoseSnapshot = tuneStartPose;
    mode = tuneRunMode;
    testCase = tuneTestCase;
    relocalize = relocalizationSummary;
    tuneMutex.give();
    const TuneTestDefinition test = tuneTestDefinition(testCase);

    std::string report;
    appendFormat(report, "=== LOCALIZATION TUNE REPORT ===\n");
    appendFormat(report, "time_ms=%lu\n", static_cast<unsigned long>(pros::millis()));
    appendFormat(report, "status=%s\n", status);
    appendFormat(report, "step=%s\n", stepName);
    appendFormat(report, "mode=%s\n", tuneRunModeName(mode));
    appendFormat(report, "test_case=%d %s\n", tuneTestCaseNumber(testCase), test.name);
    appendFormat(report, "test_objective=%s\n", test.objective);
    appendFormat(report, "output_mode=%s\n", tuneLogOutputModeName(outputMode));
    const lemlib::Pose startPose =
        checkpointCount > 0 && checkpoints[0].recorded ? checkpoints[0].expected : startPoseSnapshot;
    appendFormat(report, "start_pose=%.1f %.1f %.1f\n", startPose.x, startPose.y, lemlib::radToDeg(startPose.theta));
    appendFormat(report, "relocalize_attempted=%d\n", relocalize.attempted);
    appendFormat(report, "relocalize_success=%d\n", relocalize.success);
    appendFormat(report, "relocalize_status=%s\n", relocalize.status);
    appendFormat(report, "relocalize_pose=%.1f %.1f %.1f\n", relocalize.pose.x, relocalize.pose.y,
                 lemlib::radToDeg(relocalize.pose.theta));
    appendFormat(report, "relocalize_heading mode=%s imu=%.1f hypotheses=%d\n", relocalize.headingMode,
                 lemlib::radToDeg(relocalize.imuHeading), relocalize.headingHypotheses);
    appendFormat(report, "relocalize_conf=%.3f active=%d scored=%d iter=%d\n", relocalize.confidence,
                 relocalize.activeSensors, relocalize.scoredSensors, relocalize.iterations);
    appendFormat(report, "relocalize_residual=%.3f %.3f\n", relocalize.meanResidual, relocalize.maxResidual);
    appendFormat(report, "relocalize_var=%.3f %.3f %.5f\n", relocalize.varX, relocalize.varY,
                 relocalize.varTheta);
    appendFormat(report, "relocalize_sensors F %.1f/%d R %.1f/%d\n", relocalize.distances.front.distance,
                 relocalize.distances.front.confidence, relocalize.distances.right.distance,
                 relocalize.distances.right.confidence);
    appendFormat(report, "relocalize_sensors B %.1f/%d L %.1f/%d\n", relocalize.distances.back.distance,
                 relocalize.distances.back.confidence, relocalize.distances.left.distance,
                 relocalize.distances.left.confidence);
    appendFormat(report, "route_step_count=%lu\n", static_cast<unsigned long>(test.stepCount));
    for (size_t i = 0; i < test.stepCount; ++i) {
        const AutonomousRouteStep& step = test.steps[i];
        const lemlib::Pose target = autonomousRouteTarget(startPose, step);
        appendFormat(report,
                     "route_step%lu=%s type=%s local=%.1f %.1f heading_offset=%.1f abs=%.1f %.1f %.1f timeout=%d max=%.0f\n",
                     static_cast<unsigned long>(i + 1), step.name, tuneMotionKindName(step.kind), step.localRight,
                     step.localForward,
                     step.headingOffsetDeg, target.x, target.y, lemlib::radToDeg(target.theta), step.timeoutMs,
                     step.maxSpeed);
    }
    appendFormat(report, "checkpoint_count=%d\n\n", checkpointCount);
    if (outputMode == TuneLogOutputMode::DeepDive) {
        appendLocalizationConfig(report);
        appendFormat(report, "\n");
    }

    for (int i = 0; i < checkpointCount; ++i) {
        const TuneCheckpoint& checkpoint = checkpoints[i];
        const float expectedTheta = lemlib::radToDeg(checkpoint.expected.theta);
        const float reportedTheta = lemlib::radToDeg(checkpoint.reported.theta);
        const float fusedTheta = lemlib::radToDeg(checkpoint.debug.fusedPose.theta);
        const float mclTheta = lemlib::radToDeg(checkpoint.debug.mclPose.theta);
        const float errorX = checkpoint.reported.x - checkpoint.expected.x;
        const float errorY = checkpoint.reported.y - checkpoint.expected.y;
        const float errorTheta = wrapDegrees(reportedTheta - expectedTheta);

        appendFormat(report, "[%d] %s\n", i + 1, checkpoint.name);
        appendFormat(report, "Exp %.1f %.1f %.1f\n", checkpoint.expected.x, checkpoint.expected.y, expectedTheta);
        appendFormat(report, "Rep %.1f %.1f %.1f\n", checkpoint.reported.x, checkpoint.reported.y, reportedTheta);
        appendFormat(report, "Err %.1f %.1f %.1f\n", errorX, errorY, errorTheta);
        appendFormat(report, "Seq %lu\n", static_cast<unsigned long>(checkpoint.odomSeq));
        appendFormat(report, "F %.1f/%d R %.1f/%d\n", checkpoint.distances.front.distance,
                     checkpoint.distances.front.confidence, checkpoint.distances.right.distance,
                     checkpoint.distances.right.confidence);
        appendFormat(report, "B %.1f/%d L %.1f/%d\n", checkpoint.distances.back.distance,
                     checkpoint.distances.back.confidence, checkpoint.distances.left.distance,
                     checkpoint.distances.left.confidence);
        appendFormat(report, "A%d C%.2f OK%d\n", checkpoint.debug.activeSensors, checkpoint.debug.mclConfidence,
                     checkpoint.debug.correctionAccepted);
        appendFormat(report, "Gate mask=%lu dxy=%.2f dth=%.2f stable=%d/%d inl=%d res=%.2f/%.2f\n",
                     static_cast<unsigned long>(checkpoint.debug.correctionRejectMask),
                     checkpoint.debug.measurementPoseDelta, checkpoint.debug.measurementHeadingDelta,
                     checkpoint.debug.candidateStableScans, checkpoint.debug.requiredStableScans,
                     checkpoint.debug.correctionInlierSensors, checkpoint.debug.correctionMeanResidual,
                     checkpoint.debug.correctionMaxResidual);
        appendFormat(report, "Fus %.1f %.1f %.1f\n", checkpoint.debug.fusedPose.x, checkpoint.debug.fusedPose.y,
                     fusedTheta);
        appendFormat(report, "MCL %.1f %.1f %.1f\n", checkpoint.debug.mclPose.x, checkpoint.debug.mclPose.y,
                     mclTheta);
        appendFormat(report, "NIS %.2f ESS %.1f\n", checkpoint.debug.lastNis, checkpoint.debug.mclEss);
        appendFormat(report, "Var %.2f %.2f\n", checkpoint.debug.mclVarX, checkpoint.debug.mclVarY);
        appendFormat(report, "ThVar %.3f Stale%d\n\n", checkpoint.debug.mclVarTheta,
                     checkpoint.debug.sensorsStale);
    }

    return report;
}

std::string buildTuneTraceCsv() {
    const auto traceSamples = lemlib::localization::getTraceSamples(false);
    const auto config = lemlib::localization::getConfig();
    RelocalizationSummary relocalize {};
    TuneTestCase testCase = kDefaultTuneTestCase;

    tuneMutex.take();
    relocalize = relocalizationSummary;
    testCase = tuneTestCase;
    tuneMutex.give();
    const TuneTestDefinition test = tuneTestDefinition(testCase);

    std::string csv;
    appendFormat(csv, "# localization_trace_v2\n");
    appendFormat(csv, "# test_case,%d,%s,%s\n", tuneTestCaseNumber(testCase), test.name, test.objective);
    appendFormat(csv, "# relocalize,%d,%d,%s,%.4f,%.4f,%.4f,%.6f,%.6f,%.8f,%d,%d\n", relocalize.attempted,
                 relocalize.success, relocalize.status, relocalize.pose.x, relocalize.pose.y,
                 lemlib::radToDeg(relocalize.pose.theta), relocalize.confidence, relocalize.varX, relocalize.varY,
                 relocalize.activeSensors, relocalize.iterations);
    appendFormat(csv, "# relocalize_residual,%.6f,%.6f,%d\n", relocalize.meanResidual, relocalize.maxResidual,
                 relocalize.scoredSensors);
    appendFormat(csv, "# relocalize_heading,%s,%.4f,%d\n", relocalize.headingMode,
                 lemlib::radToDeg(relocalize.imuHeading), relocalize.headingHypotheses);
    appendFormat(csv, "# ekf_period_ms=%lu,mcl_period_ms=%lu,nis_gate=%.3f,min_correction_sensors=%d,min_confidence=%.3f,blend=%.3f\n",
                 static_cast<unsigned long>(config.fusion.ekfPeriodMs),
                 static_cast<unsigned long>(config.fusion.mclPeriodMs), config.fusion.nisGate,
                 config.fusion.minCorrectionSensors, config.fusion.minConfidence, config.fusion.blend);
    appendFormat(csv, "# max_var_xy=%.3f,max_var_theta=%.5f,max_meas_xy=%.3f,max_meas_theta_deg=%.3f,max_corr_xy=%.3f,max_corr_theta_deg=%.3f,stale_ms=%lu\n",
                 config.fusion.maxVarXY, config.fusion.maxVarTheta, config.fusion.maxMeasurementDeltaXY,
                 lemlib::radToDeg(config.fusion.maxMeasurementDeltaTheta), config.fusion.maxCorrectionXY,
                 lemlib::radToDeg(config.fusion.maxCorrectionTheta),
                 static_cast<unsigned long>(config.fusion.sensorStaleMs));
    appendFormat(csv, "# field_bounds,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", config.field.minX, config.field.maxX,
                 config.field.minY, config.field.maxY, config.field.fieldMargin, config.field.obstacleMargin);
    appendFormat(csv, "# odom_model,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", vertical.getOffset(),
                 horizontal.getOffset(), drivetrain.trackWidth, vertical.getWheelDiameter(),
                 vertical.getGearRatio(), horizontal.getWheelDiameter(), horizontal.getGearRatio());
    for (size_t i = 0; i < config.field.obstacles.size(); ++i) {
        const auto& obstacle = config.field.obstacles[i];
        const char* type =
            obstacle.type == lemlib::localization::FieldConfig::Obstacle::Type::Circle ? "circle" : "rect";
        appendFormat(csv, "# obstacle,%lu,%s,%.4f,%.4f,%.4f,%.4f,%.4f\n", static_cast<unsigned long>(i), type,
                     obstacle.x, obstacle.y, obstacle.radius, obstacle.halfW, obstacle.halfH);
    }
    for (size_t i = 0; i < config.sensors.size(); ++i) {
        const auto& sensor = config.sensors[i];
        appendFormat(csv, "# sensor_model,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n", static_cast<unsigned long>(i),
                     sensor.dx, sensor.dy, lemlib::radToDeg(sensor.dtheta), sensor.minRange, sensor.maxRange,
                     sensor.minConfidence);
    }
    appendFormat(
        csv,
        "time_ms,odom_seq,odom_x,odom_y,odom_theta_deg,odom_only_x,odom_only_y,odom_only_theta_deg,odom_local_x,odom_local_y,odom_delta_theta_deg,odom_tel_seq,odom_tel_dt,odom_heading_before_deg,odom_heading_after_deg,odom_delta_heading_deg,odom_heading_horizontal_pair,odom_heading_vertical_pair,odom_heading_imu,odom_heading_fallback,odom_vertical1_raw,odom_vertical2_raw,odom_horizontal1_raw,odom_horizontal2_raw,odom_imu_raw_deg,odom_delta_vertical1,odom_delta_vertical2,odom_delta_horizontal1,odom_delta_horizontal2,odom_delta_imu_deg,odom_selected_vertical_raw,odom_selected_horizontal_raw,odom_selected_delta_vertical,odom_selected_delta_horizontal,odom_selected_vertical_offset,odom_selected_horizontal_offset,ekf_x,ekf_y,ekf_theta_deg,applied_x,applied_y,applied_theta_deg,target_corr_x,target_corr_y,target_corr_theta_deg,applied_corr_x,applied_corr_y,applied_corr_theta_deg,mcl_x,mcl_y,mcl_theta_deg,mcl_conf,mcl_ess,last_nis,mcl_var_x,mcl_var_y,mcl_var_theta,active_sensors,mcl_valid,sensors_stale,correction_accepted,correction_reject_mask,meas_delta_xy,meas_delta_theta_deg,candidate_inlier_sensors,candidate_mean_residual,candidate_max_residual,candidate_stable_scans,required_stable_scans");
    for (const char* sensorName : kTraceSensorNames) {
        appendFormat(csv,
                     ",%s_dist,%s_conf,%s_in_range,%s_conf_ok,%s_used,%s_size,%s_velocity,%s_expected,%s_residual,%s_age_ms,%s_reading_seq,%s_changed",
                     sensorName, sensorName, sensorName, sensorName, sensorName, sensorName, sensorName, sensorName,
                     sensorName, sensorName, sensorName, sensorName);
    }
    appendFormat(csv, "\n");

    for (const auto& sample : traceSamples) {
        appendFormat(
            csv,
            "%lu,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.8f,%d,%d,%d,%d,%lu,%.4f,%.4f,%d,%.4f,%.4f,%d,%d",
            static_cast<unsigned long>(sample.timeMs), static_cast<unsigned long>(sample.odomSeq),
            sample.odomPose.x, sample.odomPose.y, sample.odomPose.theta, sample.odomOnlyPose.x,
            sample.odomOnlyPose.y, sample.odomOnlyPose.theta, sample.odomDelta.localX, sample.odomDelta.localY,
            sample.odomDelta.deltaTheta,
            static_cast<unsigned long>(sample.odomTelemetry.seq), sample.odomTelemetry.dt,
            sample.odomTelemetry.headingBefore, sample.odomTelemetry.headingAfter,
            sample.odomTelemetry.deltaHeading, sample.odomTelemetry.usedHorizontalHeadingPair,
            sample.odomTelemetry.usedVerticalHeadingPair, sample.odomTelemetry.usedImuHeading,
            sample.odomTelemetry.usedHeadingFallback, sample.odomTelemetry.vertical1Raw,
            sample.odomTelemetry.vertical2Raw, sample.odomTelemetry.horizontal1Raw,
            sample.odomTelemetry.horizontal2Raw, sample.odomTelemetry.imuRaw,
            sample.odomTelemetry.deltaVertical1, sample.odomTelemetry.deltaVertical2,
            sample.odomTelemetry.deltaHorizontal1, sample.odomTelemetry.deltaHorizontal2,
            sample.odomTelemetry.deltaImu, sample.odomTelemetry.selectedVerticalRaw,
            sample.odomTelemetry.selectedHorizontalRaw, sample.odomTelemetry.selectedDeltaVertical,
            sample.odomTelemetry.selectedDeltaHorizontal, sample.odomTelemetry.selectedVerticalOffset,
            sample.odomTelemetry.selectedHorizontalOffset, sample.ekfPose.x, sample.ekfPose.y,
            sample.ekfPose.theta, sample.appliedPose.x, sample.appliedPose.y, sample.appliedPose.theta,
            sample.targetCorrectionX, sample.targetCorrectionY, sample.targetCorrectionTheta,
            sample.appliedCorrectionX, sample.appliedCorrectionY, sample.appliedCorrectionTheta,
            sample.mclPose.x, sample.mclPose.y, sample.mclPose.theta, sample.mclConfidence, sample.mclEss,
            sample.lastNis, sample.mclVarX, sample.mclVarY, sample.mclVarTheta, sample.activeSensors,
            sample.mclValid, sample.sensorsStale, sample.correctionAccepted,
            static_cast<unsigned long>(sample.correctionRejectMask), sample.measurementPoseDelta,
            sample.measurementHeadingDelta, sample.correctionInlierSensors, sample.correctionMeanResidual,
            sample.correctionMaxResidual, sample.candidateStableScans, sample.requiredStableScans);
        for (const auto& sensor : sample.sensors) {
            appendFormat(csv, ",%.4f,%d,%d,%d,%d,%d,%.6f,%.4f,%.4f,%u,%u,%d", sensor.distance, sensor.confidence,
                         sensor.inRange, sensor.confidenceAccepted, sensor.used, sensor.objectSize,
                         sensor.objectVelocity, sensor.expectedDistance, sensor.residual,
                         static_cast<unsigned int>(sensor.readingAgeMs), static_cast<unsigned int>(sensor.readingSeq),
                         sensor.readingChanged);
        }
        appendFormat(csv, "\n");
    }

    return csv;
}

std::string buildTuneLogFile() {
    std::string log = buildTuneReport();
    if (kTuneLogOutputMode == TuneLogOutputMode::DeepDive) {
        appendFormat(log, "=== LOCALIZATION TRACE CSV ===\n");
        log.append(buildTuneTraceCsv());
    }
    return log;
}

bool writeTextFile(const char* path, const std::string& contents) {
    std::FILE* file = std::fopen(path, "w");
    if (file == nullptr) return false;
    const size_t written = std::fwrite(contents.data(), sizeof(char), contents.size(), file);
    std::fclose(file);
    return written == contents.size();
}

bool readTextFile(const char* path, std::string& contents) {
    std::FILE* file = std::fopen(path, "r");
    if (file == nullptr) return false;
    if (std::fseek(file, 0, SEEK_END) != 0) {
        std::fclose(file);
        return false;
    }

    const long size = std::ftell(file);
    if (size < 0 || std::fseek(file, 0, SEEK_SET) != 0) {
        std::fclose(file);
        return false;
    }

    std::string buffer(static_cast<size_t>(size), '\0');
    const size_t read = std::fread(buffer.data(), sizeof(char), buffer.size(), file);
    std::fclose(file);
    if (read != buffer.size()) return false;
    contents = std::move(buffer);
    return true;
}

void persistTuneLogInternal(const std::string& fullLog) { writeTextFile(kTuneInternalLogPath, fullLog); }

void restoreInternalTuneLogIfPresent() {
    std::string persistedLog;
    if (!readTextFile(kTuneInternalLogPath, persistedLog) || persistedLog.empty()) return;
    cachePendingTuneTerminalLog(persistedLog);
    setTuneExportFeedback("Recovered internal log", 0);
}

void streamTextToTerminal(const std::string& text) {
    constexpr size_t kChunkSize = 512;
    constexpr std::uint32_t kChunkDelayMs = 2;
    const int serialFd = ::open("serial", O_RDWR);
    if (serialFd >= 0) {
        pros::c::fdctl(serialFd, SERCTL_ACTIVATE, reinterpret_cast<void*>(kStdoutStreamId));
        pros::c::fdctl(serialFd, SERCTL_BLKWRITE, nullptr);
    }

    for (size_t pos = 0; pos < text.size(); pos += kChunkSize) {
        const size_t count = std::min(kChunkSize, text.size() - pos);
        std::printf("%.*s", static_cast<int>(count), text.data() + pos);
        std::fflush(stdout);
        if (serialFd >= 0) ::write(serialFd, text.data() + pos, count);
        pros::delay(kChunkDelayMs);
    }

    if (serialFd >= 0) ::close(serialFd);
}

void ensureTerminalStdoutActive() {
    pros::c::serctl(SERCTL_ACTIVATE, reinterpret_cast<void*>(kStdoutStreamId));
}

void streamTuneLogBlockToTerminal(const std::string& fullLog) {
    constexpr const char* kTerminalBeginMarker = "=== BEGIN LOCALIZATION TUNE LOG ===\n";
    constexpr const char* kTerminalEndMarker = "\n=== END LOCALIZATION TUNE LOG ===\n";
    ensureTerminalStdoutActive();
    streamTextToTerminal(kTerminalBeginMarker);
    streamTextToTerminal(fullLog);
    streamTextToTerminal(kTerminalEndMarker);
}

void cachePendingTuneTerminalLog(const std::string& fullLog) {
    pendingTuneLogMutex.take();
    pendingTuneTerminalLog = fullLog;
    pendingTuneTerminalReplay = !fullLog.empty();
    pendingTuneTerminalStreaming = false;
    pendingTuneTerminalDumpRequested = false;
    pendingTuneLogMutex.give();
}

void clearPendingTuneTerminalLog() {
    pendingTuneLogMutex.take();
    pendingTuneTerminalLog.clear();
    pendingTuneTerminalReplay = false;
    pendingTuneTerminalStreaming = false;
    pendingTuneTerminalDumpRequested = false;
    pendingTuneLogMutex.give();
    clearExportFeedback();
}

bool hasPendingTuneTerminalReplay() {
    bool ready = false;
    pendingTuneLogMutex.take();
    ready = pendingTuneTerminalReplay && !pendingTuneTerminalLog.empty();
    pendingTuneLogMutex.give();
    return ready;
}

bool hasLiveTuneData() {
    bool hasData = false;
    tuneMutex.take();
    hasData = tuneTestRunning || tuneCheckpointCount > 0 || relocalizationSummary.attempted ||
              tuneRunMode != TuneRunMode::Idle || (tuneStatus != nullptr && std::string_view(tuneStatus) != "Idle");
    tuneMutex.give();
    if (hasData) return true;
    return !lemlib::localization::getTraceSamples(false).empty();
}

bool hasTuneExportAvailable() { return hasPendingTuneTerminalReplay() || hasLiveTuneData(); }

bool queueTuneDataExportToTerminal() {
    pendingTuneLogMutex.take();
    const bool hasCachedLog = pendingTuneTerminalReplay && !pendingTuneTerminalLog.empty();
    pendingTuneLogMutex.give();

    if (!hasCachedLog) {
        if (!hasLiveTuneData()) return false;
        cachePendingTuneTerminalLog(buildTuneLogFile());
    }

    pendingTuneLogMutex.take();
    pendingTuneTerminalDumpRequested = pendingTuneTerminalReplay && !pendingTuneTerminalLog.empty();
    const bool queued = pendingTuneTerminalDumpRequested;
    pendingTuneLogMutex.give();
    return queued;
}

void terminalReplayTaskFn() {
    while (true) {
        std::string replayLog;

        pendingTuneLogMutex.take();
        const bool shouldReplay = pendingTuneTerminalReplay && pendingTuneTerminalDumpRequested &&
                                  !pendingTuneTerminalStreaming && !pendingTuneTerminalLog.empty();
        if (shouldReplay) {
            pendingTuneTerminalStreaming = true;
            pendingTuneTerminalDumpRequested = false;
            replayLog = pendingTuneTerminalLog;
        }
        pendingTuneLogMutex.give();

        if (!replayLog.empty()) {
            setTuneExportFeedback("Exporting tune log...", 0);
            streamTuneLogBlockToTerminal(replayLog);

            pendingTuneLogMutex.take();
            pendingTuneTerminalStreaming = false;
            pendingTuneTerminalReplay = !pendingTuneTerminalLog.empty();
            pendingTuneLogMutex.give();

            setTuneExportFeedback("USB dump attempted", kTuneExportFeedbackMs);
        }

        pros::delay(50);
    }
}

void emitTuneReport() {
    const std::string fullLog = buildTuneLogFile();

    if (pros::usd::is_installed() != 1) {
        cachePendingTuneTerminalLog(fullLog);
        persistTuneLogInternal(fullLog);
        setTuneExportFeedback("Log cached in RAM", 0);
        lemlib::bufferedStdout().print(
            "No SD card detected; cached tune log in RAM. After plugging in later, press Y or tap the brain to "
            "dump it\n");
        return;
    }
    if (!writeTextFile(kTuneLogPath, fullLog)) {
        cachePendingTuneTerminalLog(fullLog);
        persistTuneLogInternal(fullLog);
        setTuneExportFeedback("Log cached in RAM", 0);
        lemlib::bufferedStdout().print(
            "Tune report write failed: {}; cached tune log in RAM. Press Y or tap the brain to dump it\n",
            kTuneLogPath);
    } else {
        clearPendingTuneTerminalLog();
        const std::string report = buildTuneReport();
        lemlib::bufferedStdout().print("\n{}\n", report);
        lemlib::bufferedStdout().print("Tune report saved: {}\n", kTuneLogPath);
    }
}

bool exportTuneDataToTerminalNow() {
    if (!queueTuneDataExportToTerminal()) return false;
    setTuneExportFeedback("Export queued...", kTuneExportFeedbackMs);
    return true;
}

void storeTuneCheckpoint(const char* name, const lemlib::Pose& expected) {
    TuneCheckpoint checkpoint {};
    checkpoint.name = name;
    checkpoint.expected = expected;
    const auto traceSample = lemlib::localization::getLatestTraceSample(true);
    if (traceSample.timeMs != 0) {
        checkpoint.reported = traceSample.appliedPose;
        checkpoint.debug.fusedPose = traceSample.ekfPose;
        checkpoint.debug.mclPose = traceSample.mclPose;
        checkpoint.debug.mclConfidence = traceSample.mclConfidence;
        checkpoint.debug.mclEss = traceSample.mclEss;
        checkpoint.debug.lastNis = traceSample.lastNis;
        checkpoint.debug.mclVarX = traceSample.mclVarX;
        checkpoint.debug.mclVarY = traceSample.mclVarY;
        checkpoint.debug.mclVarTheta = traceSample.mclVarTheta;
        checkpoint.debug.activeSensors = traceSample.activeSensors;
        checkpoint.debug.mclValid = traceSample.mclValid;
        checkpoint.debug.sensorsStale = traceSample.sensorsStale;
        checkpoint.debug.correctionAccepted = traceSample.correctionAccepted;
        checkpoint.debug.correctionRejectMask = traceSample.correctionRejectMask;
        checkpoint.debug.measurementPoseDelta = traceSample.measurementPoseDelta;
        checkpoint.debug.measurementHeadingDelta = traceSample.measurementHeadingDelta;
        checkpoint.debug.correctionInlierSensors = traceSample.correctionInlierSensors;
        checkpoint.debug.correctionMeanResidual = traceSample.correctionMeanResidual;
        checkpoint.debug.correctionMaxResidual = traceSample.correctionMaxResidual;
        checkpoint.debug.candidateStableScans = traceSample.candidateStableScans;
        checkpoint.debug.requiredStableScans = traceSample.requiredStableScans;
        checkpoint.distances = captureDistances(traceSample);
        checkpoint.odomSeq = traceSample.odomSeq;
    } else {
        checkpoint.reported = chassis.getPose(true);
        checkpoint.debug = lemlib::localization::getDebugInfo(true);
        checkpoint.distances = captureDistances();
    }
    checkpoint.recorded = true;

    tuneMutex.take();
    const int slot =
        (tuneCheckpointCount < static_cast<int>(tuneCheckpoints.size())) ? tuneCheckpointCount :
                                                                           static_cast<int>(tuneCheckpoints.size()) - 1;
    tuneCheckpoints[slot] = checkpoint;
    if (tuneCheckpointCount < static_cast<int>(tuneCheckpoints.size())) ++tuneCheckpointCount;
    tuneSelectedCheckpoint = slot;
    tuneMutex.give();

    const float errorX = checkpoint.reported.x - expected.x;
    const float errorY = checkpoint.reported.y - expected.y;
    const float errorTheta = wrapDegrees(lemlib::radToDeg(checkpoint.reported.theta - expected.theta));
    lemlib::telemetrySink()->info(
        "Tune checkpoint {} expected={} reported={} err=({:.2f}, {:.2f}, {:.2f} deg) active={} conf={:.2f} accept={}",
        name, expected, checkpoint.reported, errorX, errorY, errorTheta, checkpoint.debug.activeSensors,
        checkpoint.debug.mclConfidence, checkpoint.debug.correctionAccepted);
}

bool waitForTuneMotionCompletion(TuneMotionKind kind, const lemlib::Pose& expected, int timeoutMs, bool allowAbort) {
    (void)allowAbort;
    (void)kind;
    (void)expected;
    const std::uint32_t startedAt = pros::millis();
    bool sawMotionRunning = false;

    while (pros::millis() - startedAt < static_cast<std::uint32_t>(std::max(timeoutMs, 0))) {
        const bool motionRunning = chassis.isInMotion();
        sawMotionRunning = sawMotionRunning || motionRunning;

        if (sawMotionRunning && !motionRunning) return true;
        pros::delay(kTuneMotionPollMs);
    }

    if (chassis.isInMotion()) {
        chassis.cancelMotion();
        pros::delay(kTuneMotionCancelSettleMs);
    }
    return true;
}

bool settleAndCapture(const char* name, const lemlib::Pose& expected, bool allowAbort) {
    (void)allowAbort;
    setState("Settling", name, true);
    pros::delay(kTuneSettleMs);
    storeTuneCheckpoint(name, expected);
    return true;
}

bool waitForPostMoveLocalizationUpdate(bool allowAbort) {
    (void)allowAbort;
    const std::uint32_t startedAt = pros::millis();

    while (pros::millis() - startedAt < kTunePostMoveCorrectionWindowMs) {
        const auto debug = lemlib::localization::getDebugInfo(true);
        const auto sample = lemlib::localization::getLatestTraceSample(true);
        const bool freshAfterMotion = sample.timeMs >= startedAt &&
                                      (sample.correctionRejectMask & kTuneRejectMotionSuppressedMask) == 0;
        const bool goodMeasurement =
            freshAfterMotion && debug.activeSensors >= 2 && debug.lastNis >= 0.0f &&
            debug.lastNis <= kTunePostMoveGoodNis && debug.correctionInlierSensors >= kTunePostMoveMinInlierSensors &&
            debug.correctionMaxResidual <= kTunePostMoveMaxResidual && !debug.sensorsStale;
        if ((freshAfterMotion && debug.correctionAccepted) || goodMeasurement) return true;

        pros::delay(kTuneMotionPollMs);
    }

    return true;
}

bool runMovePoseStep(const char* name, const lemlib::Pose& expected, int timeout,
                     const lemlib::MoveToPoseParams& params, bool allowAbort) {
    setState("Move", name, true);
    // Chassis::moveToPose accepts LemLib field-heading degrees (0 = field +Y)
    // and converts to its internal standard-position heading itself.
    chassis.moveToPose(expected.x, expected.y, lemlib::radToDeg(expected.theta), timeout, params, true);
    if (!waitForTuneMotionCompletion(TuneMotionKind::Move, expected, timeout, allowAbort)) return false;
    if (!waitForPostMoveLocalizationUpdate(allowAbort)) return false;
    return settleAndCapture(name, expected, allowAbort);
}

bool runTurnStep(const char* name, const lemlib::Pose& expected, int timeout,
                 const lemlib::TurnToHeadingParams& params, bool allowAbort) {
    setState("Turn", name, true);
    chassis.turnToHeading(lemlib::radToDeg(expected.theta), timeout, params, true);
    if (!waitForTuneMotionCompletion(TuneMotionKind::Turn, expected, timeout, allowAbort)) return false;
    if (!waitForPostMoveLocalizationUpdate(allowAbort)) return false;
    return settleAndCapture(name, expected, allowAbort);
}

bool runDriveProbeStep(const char* name, const lemlib::Pose& expected, int durationMs, float power, bool forwards,
                       bool allowAbort) {
    setState("Drive", name, true);
    // Open-loop: equal raw power to both sides, drive curve disabled, no PID. The
    // expected heading is the start heading, so the captured heading error is the
    // pure drivetrain veer over this segment.
    chassis.cancelMotion();
    const int command = static_cast<int>(forwards ? power : -power);
    const std::uint32_t startedAt = pros::millis();
    while (pros::millis() - startedAt < static_cast<std::uint32_t>(durationMs)) {
        chassis.tank(command, command, true);
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
    pros::delay(150); // let the robot fully stop and odom settle before capture
    return settleAndCapture(name, expected, allowAbort);
}

bool beginTuneFinalization(const char* status, const char* stepName) {
    bool shouldFinalize = false;
    tuneMutex.take();
    if (tuneTestRunning) {
        tuneStatus = status;
        tuneStepName = stepName;
        tuneTestRunning = false;
        shouldFinalize = true;
    }
    tuneMutex.give();
    return shouldFinalize;
}

void printCheckpointPage(const TuneCheckpoint& checkpoint, int index, int count, const char* status, int page) {
    const float expectedTheta = lemlib::radToDeg(checkpoint.expected.theta);
    const float reportedTheta = lemlib::radToDeg(checkpoint.reported.theta);
    const float errorX = checkpoint.reported.x - checkpoint.expected.x;
    const float errorY = checkpoint.reported.y - checkpoint.expected.y;
    const float errorTheta = wrapDegrees(reportedTheta - expectedTheta);
    const bool logReady = hasTuneExportAvailable();
    const char* exportFeedback = currentTuneExportFeedback();

    pros::screen::print(TEXT_MEDIUM, 1, "Tune %s %d/%d", status, index + 1, count);
    pros::screen::print(TEXT_MEDIUM, 2, "%s", checkpoint.name);
    if (page == 0) {
        pros::screen::print(TEXT_MEDIUM, 3, "Exp %.1f %.1f %.1f", checkpoint.expected.x, checkpoint.expected.y,
                            expectedTheta);
        pros::screen::print(TEXT_MEDIUM, 4, "Rep %.1f %.1f %.1f", checkpoint.reported.x, checkpoint.reported.y,
                            reportedTheta);
        pros::screen::print(TEXT_MEDIUM, 5, "Err %.1f %.1f %.1f", errorX, errorY, errorTheta);
        pros::screen::print(TEXT_MEDIUM, 6, "F %.1f/%d R %.1f/%d", checkpoint.distances.front.distance,
                            checkpoint.distances.front.confidence, checkpoint.distances.right.distance,
                            checkpoint.distances.right.confidence);
        pros::screen::print(TEXT_MEDIUM, 7, "B %.1f/%d L %.1f/%d", checkpoint.distances.back.distance,
                            checkpoint.distances.back.confidence, checkpoint.distances.left.distance,
                            checkpoint.distances.left.confidence);
        if (exportFeedback != nullptr) {
            pros::screen::print(TEXT_MEDIUM, 8, "%s", exportFeedback);
        } else {
            pros::screen::print(TEXT_MEDIUM, 8, logReady ? "LOG READY tap LR dump" : "A%d C%.2f OK%d",
                                checkpoint.debug.activeSensors, checkpoint.debug.mclConfidence,
                                checkpoint.debug.correctionAccepted);
        }
    } else {
        const float fusedTheta = lemlib::radToDeg(checkpoint.debug.fusedPose.theta);
        const float mclTheta = lemlib::radToDeg(checkpoint.debug.mclPose.theta);
        pros::screen::print(TEXT_MEDIUM, 3, "Fus %.1f %.1f %.1f", checkpoint.debug.fusedPose.x,
                            checkpoint.debug.fusedPose.y, fusedTheta);
        pros::screen::print(TEXT_MEDIUM, 4, "MCL %.1f %.1f %.1f", checkpoint.debug.mclPose.x,
                            checkpoint.debug.mclPose.y, mclTheta);
        pros::screen::print(TEXT_MEDIUM, 5, "NIS %.2f ESS %.1f", checkpoint.debug.lastNis, checkpoint.debug.mclEss);
        pros::screen::print(TEXT_MEDIUM, 6, "Var %.2f %.2f", checkpoint.debug.mclVarX, checkpoint.debug.mclVarY);
        pros::screen::print(TEXT_MEDIUM, 7, "ThVar %.3f Stale%d", checkpoint.debug.mclVarTheta,
                            checkpoint.debug.sensorsStale);
        if (exportFeedback != nullptr) pros::screen::print(TEXT_MEDIUM, 8, "%s", exportFeedback);
        else pros::screen::print(TEXT_MEDIUM, 8, logReady ? "LOG READY tap LR dump" : "Pg2 Up toggle X clr");
    }
}

void screenTaskFn() {
    std::uint32_t lastTapSequenceHandled = 0;

    while (true) {
        TuneCheckpoint checkpoint {};
        int checkpointCount = 0;
        int checkpointIndex = 0;
        int page = 0;
        const char* status = nullptr;
        const char* step = nullptr;
        bool running = false;
        lemlib::Pose startPose {0, 0, 0};

        tuneMutex.take();
        checkpointCount = tuneCheckpointCount;
        checkpointIndex = tuneSelectedCheckpoint;
        page = tuneDisplayPage;
        status = tuneStatus;
        step = tuneStepName;
        running = tuneTestRunning;
        startPose = tuneStartPose;
        if (checkpointCount > 0) {
            if (checkpointIndex < 0) checkpointIndex = 0;
            if (checkpointIndex >= checkpointCount) checkpointIndex = checkpointCount - 1;
            checkpoint = tuneCheckpoints[checkpointIndex];
        }
        tuneMutex.give();

        const bool logReady = hasTuneExportAvailable();
        const bool autonomousActive = pros::competition::is_autonomous() != 0;
        const bool disabledActive = pros::competition::is_disabled() != 0;
        const char* modeLabel =
            autonomousActive ? "AUTO" :
            (driverControlLoopActive ? "DRIVER" :
            (manualDriveFallbackActive ? "MANUAL" :
            (disabledActive ? "DISABLED" : "IDLE")));
        const int controllerConnected = pros::c::controller_is_connected(pros::E_CONTROLLER_MASTER);
        const int leftY = pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int rightX =
            pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X);
        const bool exportTapCallback = tuneExportTapSequence != lastTapSequenceHandled;
        lastTapSequenceHandled = tuneExportTapSequence;
        if (exportTapCallback) {
            if (logReady) {
                if (exportTuneDataToTerminalNow()) {
                    setTuneExportFeedback("Export queued...", kTuneExportFeedbackMs);
                } else {
                    setTuneExportFeedback("Export failed", kTuneExportFeedbackMs);
                    setState("Idle", "Export failed", false);
                }
            } else {
                setTuneExportFeedback("No tune data ready", kTuneExportFeedbackMs);
                setState("Idle", "No tune data ready", false);
            }
        }

        pros::screen::erase();
        if (checkpointCount == 0) {
            const lemlib::Pose pose = chassis.getPose();
            const float startHeading = lemlib::radToDeg(startPose.theta);
            const DistanceSnapshot distances = captureDistances();
            pros::screen::print(TEXT_MEDIUM, 1, "Mode %s CTL %d", modeLabel, controllerConnected);
            pros::screen::print(TEXT_MEDIUM, 2, "Tune %s", status);
            pros::screen::print(TEXT_MEDIUM, 3, "%s", step);
            pros::screen::print(TEXT_MEDIUM, 4, "LY %d RX %d DL %d MF %d", leftY, rightX,
                                driverDriveLoopTicking ? 1 : 0, manualDriveFallbackActive ? 1 : 0);
            pros::screen::print(TEXT_MEDIUM, 5, "Start %.1f %.1f %.0f", startPose.x, startPose.y, startHeading);
            pros::screen::print(TEXT_MEDIUM, 6, "Pose %.1f %.1f %.1f", pose.x, pose.y, pose.theta);
            pros::screen::print(TEXT_MEDIUM, 7, "F %.1f/%d R %.1f/%d", distances.front.distance,
                                distances.front.confidence, distances.right.distance, distances.right.confidence);
            const char* exportFeedback = currentTuneExportFeedback();
            if (exportFeedback != nullptr) {
                pros::screen::print(TEXT_MEDIUM, 8, "%s", exportFeedback);
            } else {
                pros::screen::print(TEXT_MEDIUM, 8, logReady ? "Tap lower-right to dump" : "B %.1f/%d L %.1f/%d",
                                    distances.back.distance, distances.back.confidence, distances.left.distance,
                                    distances.left.confidence);
            }
        } else {
            printCheckpointPage(checkpoint, checkpointIndex, checkpointCount, status, page);
        }

        if (running) pros::screen::print(TEXT_MEDIUM, 8, "Autonomous run active");
        pros::delay(50);
    }
}

void overlayControlTaskFn() {
    while (true) {
        bool tuneRunning = false;
        TuneRunMode runMode = TuneRunMode::Idle;

        tuneMutex.take();
        tuneRunning = tuneTestRunning;
        runMode = tuneRunMode;
        tuneMutex.give();

        const bool autonomousActive = pros::competition::is_autonomous() != 0;
        if (tuneRunning && runMode == TuneRunMode::AutonomousRelocalized && !autonomousActive) {
            finalizeInterruptedRunIfNeeded("Interrupted", "Autonomous ended outside callback");
            pros::delay(20);
            continue;
        }
        pros::delay(20);
    }
}

void manualDriveFallbackTaskFn() {
    bool wasActive = false;

    while (true) {
        bool tuneRunning = false;
        bool overlayActive = false;
        tuneMutex.take();
        tuneRunning = tuneTestRunning;
        overlayActive = tuneOverlayActive;
        tuneMutex.give();

        const bool autonomousActive = pros::competition::is_autonomous() != 0;
        const bool competitionConnected = pros::competition::is_connected() != 0;
        const bool controllerConnected = pros::c::controller_is_connected(pros::E_CONTROLLER_MASTER) > 0;
        const bool shouldDrive = !competitionConnected && !driverControlLoopActive && !autonomousActive &&
                                 !tuneRunning && !overlayActive && controllerConnected;

        manualDriveFallbackActive = shouldDrive;
        if (shouldDrive) {
            const int leftY =
                pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y);
            const int rightX =
                pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X);
            const int leftDrive = std::clamp(leftY + rightX, -127, 127);
            const int rightDrive = std::clamp(leftY - rightX, -127, 127);
            commandTeleopDriveOutputs(leftDrive, rightDrive);
        } else if (wasActive) {
            commandTeleopDriveOutputs(0, 0);
        }

        wasActive = shouldDrive;
        pros::delay(10);
    }
}
} // namespace

void initializeRuntime() {
    pros::screen::touch_callback(handleTuneExportTap, pros::E_TOUCH_RELEASED);
    if (screenTask == nullptr) screenTask = new pros::Task([] { screenTaskFn(); });
    if (overlayControlTask == nullptr) overlayControlTask = new pros::Task([] { overlayControlTaskFn(); });
    if (terminalReplayTask == nullptr) terminalReplayTask = new pros::Task([] { terminalReplayTaskFn(); });
    if (manualDriveFallbackTask == nullptr) manualDriveFallbackTask = new pros::Task([] { manualDriveFallbackTaskFn(); });
    restoreInternalTuneLogIfPresent();
}

void setState(const char* status, const char* stepName, bool running) {
    tuneMutex.take();
    tuneStatus = status;
    tuneStepName = stepName;
    tuneTestRunning = running;
    tuneMutex.give();
}

void clearExportFeedback() {
    tuneMutex.take();
    tuneExportFeedback = nullptr;
    tuneExportFeedbackUntilMs = 0;
    tuneMutex.give();
}

void storeRelocalizationSummary(const RelocalizationSummary& summary) {
    tuneMutex.take();
    relocalizationSummary = summary;
    tuneMutex.give();
}

void setStartPose(const lemlib::Pose& pose) {
    tuneMutex.take();
    tuneStartPose = pose;
    tuneMutex.give();
}

void setDriverControlLoopActive(bool active) { driverControlLoopActive = active; }

void setDriverDriveLoopTicking(bool active) { driverDriveLoopTicking = active; }

void prepareAutonomousRelocalizedRun(const lemlib::Pose& currentPose) {
    clearTuneResults();
    clearRelocalizationSummary();
    clearPendingTuneTerminalLog();
    clearExportFeedback();
    lemlib::localization::clearTrace();

    tuneMutex.take();
    tuneRunMode = TuneRunMode::AutonomousRelocalized;
    tuneTestCase = TuneTestCase::NormalRoute;
    tuneStartPose = currentPose;
    tuneMutex.give();
}

RelocalizationSummary performGlobalRelocalization() {
    return autonomous_localization::performGlobalRelocalization();
}

void finalizeRun(const char* status, const char* stepName, const char* rumble) {
    chassis.cancelMotion();
    if (!beginTuneFinalization(status, stepName)) return;
    lemlib::localization::setTraceEnabled(false);
    emitTuneReport();
    if (rumble != nullptr && rumble[0] != '\0') controller.rumble(rumble);
}

void finalizeInterruptedRunIfNeeded(const char* status, const char* stepName) {
    if (!beginTuneFinalization(status, stepName)) return;

    disableEightMotorPositionHold();
    stopAutonomousManipulatorControl();
    stopReleasedPtoControls();
    chassis.cancelAllMotions();
    chassis.tank(0, 0, true);
    lemlib::localization::setTraceEnabled(false);
    lemlib::bufferedStdout().print("Finalizing interrupted tune run: {}\n", stepName);
    emitTuneReport();
    controller.rumble(". .");
}

void runAutonomousRoute(const lemlib::Pose& start, int testNumber, bool allowAbort) {
    const TuneTestCase selectedTestCase = tuneTestCaseFromNumber(testNumber);
    const TuneTestDefinition test = tuneTestDefinition(selectedTestCase);
    clearTuneResults();
    clearPendingTuneTerminalLog();
    clearRelocalizationSummary();
    clearExportFeedback();

    tuneMutex.take();
    tuneRunMode = TuneRunMode::AutonomousRelocalized;
    tuneStartPose = start;
    tuneTestCase = selectedTestCase;
    tuneMutex.give();

    setState("Setup", "Apply start pose", true);
    chassis.cancelMotion();
    lemlib::localization::clearTrace();
    lemlib::localization::setTraceEnabled(true);
    chassis.setPose(start, true);
    if (!lemlib::localization::isRunning()) lemlib::localization::start();
    else lemlib::localization::syncPose(start);
    pros::delay(kTuneInitialSettleMs);

    // Solve the absolute pose from the perimeter walls while the robot is
    // stationary at the start. Hand placement varies by several inches (most of
    // all in the forward axis), so the fixed start carries a large, variable
    // reference error that MCL cannot recover from when it is seeded tightly
    // around that wrong start. When the wall solve is accepted, commit it so odom
    // and MCL both begin from the true field pose, and shift the run's reference
    // pose to match so checkpoint expected/error values stay meaningful. Fusion
    // gates and noise models are intentionally left unchanged.
    setState("Setup", "Relocalize", true);
    const RelocalizationSummary relocalization = performGlobalRelocalization();
    storeRelocalizationSummary(relocalization);

    lemlib::Pose runStart = start;
    if (relocalization.success) {
        runStart = relocalization.pose;
        chassis.setPose(runStart, true);
        lemlib::localization::syncPose(runStart);
        setStartPose(runStart);
        pros::delay(kTuneInitialSettleMs);
    }

    if (!settleAndCapture("Start hold", runStart, allowAbort)) {
        finalizeRun("Aborted", "Stopped during start hold", ". .");
        return;
    }

    for (size_t i = 0; i < test.stepCount; ++i) {
        const AutonomousRouteStep& step = test.steps[i];
        const lemlib::Pose target = autonomousRouteTarget(runStart, step);
        bool stepOk = false;
        if (step.kind == TuneMotionKind::Turn) {
            lemlib::TurnToHeadingParams params {};
            params.maxSpeed = static_cast<int>(step.maxSpeed);
            params.minSpeed = static_cast<int>(step.minSpeed);
            params.earlyExitRange = step.earlyExitRange;
            stepOk = runTurnStep(step.name, target, step.timeoutMs, params, allowAbort);
        } else if (step.kind == TuneMotionKind::Drive) {
            stepOk = runDriveProbeStep(step.name, target, step.timeoutMs, step.maxSpeed, step.forwards, allowAbort);
        } else {
            lemlib::MoveToPoseParams params {};
            params.forwards = step.forwards;
            params.lead = step.lead;
            params.maxSpeed = step.maxSpeed;
            params.minSpeed = step.minSpeed;
            params.earlyExitRange = step.earlyExitRange;
            stepOk = runMovePoseStep(step.name, target, step.timeoutMs, params, allowAbort);
        }
        if (!stepOk) {
            finalizeRun("Aborted", step.name, ". .");
            return;
        }
    }

    finalizeRun("Complete", "Use left/right to review", "--");
}

void runAutonomousRoute(const lemlib::Pose& start, bool allowAbort) {
    runAutonomousRoute(start, tuneTestCaseNumber(kDefaultTuneTestCase), allowAbort);
}
} // namespace localization_tune
