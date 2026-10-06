#include "main.h"

#include "app_config.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/logger/stdout.hpp"
#include "localization_config.hpp"
#include "localization_tune.hpp"
#include "robot_control.hpp"

namespace {
constexpr bool kPtoEngagedValue = false;

void configureChassisPto() {
    lemlib::PtoSettings ptoSettings {};
    ptoSettings.piston = &ptoPiston;
    ptoSettings.motorGroups[0] = {.motors = &leftPtoMotor1, .driveSide = lemlib::DriveSide::LEFT};
    ptoSettings.motorGroups[1] = {.motors = &leftPtoMotor2, .driveSide = lemlib::DriveSide::LEFT};
    ptoSettings.motorGroups[2] = {.motors = &rightPtoMotor1, .driveSide = lemlib::DriveSide::RIGHT};
    ptoSettings.motorGroups[3] = {.motors = &rightPtoMotor2, .driveSide = lemlib::DriveSide::RIGHT};
    ptoSettings.engagedValue = kPtoEngagedValue;
    chassis.configurePto(ptoSettings);
}

void stopChassisMotion() {
    chassis.cancelAllMotions();
    chassis.tank(0, 0, true);
}
} // namespace

ASSET(example_txt);

void initialize() {
    if (kSmokeTestMode) {
        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "SMOKE TEST BOOT");
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "No LemLib init");
        controller.rumble(".");
        leftDriveMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        rightDriveMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        return;
    }

    // Construct the lazy logger singletons here: PROS never deletes the
    // initialize task, and no LemLib/tune task exists yet. A first construction
    // inside a competition task deleted mid-constructor would leave the
    // single-thread libstdc++ static guard "in progress", so the next call would
    // throw recursive_init_error and terminate the program.
    (void)lemlib::bufferedStdout();
    (void)lemlib::infoSink();
    (void)lemlib::telemetrySink();

    localization_tune::setState("Init", "Starting up", true);
    initializeRobotControlState();
    configureChassisPto();

    if (localization_tune::kEnabled) {
        lemlib::localization::configure(buildLocalizationConfig());
        localization_tune::initializeRuntime();
    }

    localization_tune::setState("Init", "Calibrating chassis", true);
    chassis.calibrate();

    stopChassisMotion();
    switchToFourMotorDrive();
    localization_tune::clearExportFeedback();
    localization_tune::setState("Idle", "Autonomous ready", false);
}

void disabled() {
    localization_tune::setDriverControlLoopActive(false);
    localization_tune::setDriverDriveLoopTicking(false);
    localization_tune::finalizeInterruptedRunIfNeeded("Interrupted", "Autonomous period ended");
    stopAutonomousManipulatorControl();
    stopChassisMotion();
    // If PROS killed the autonomous task mid-blocking-motion, release the
    // owner-independent motion semaphore after live cancellation has settled.
    chassis.recoverInterruptedMotion();

    if (kSmokeTestMode) {
        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "SMOKE TEST");
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Mode: Disabled");
    }
}

void competition_initialize() {
    localization_tune::setDriverControlLoopActive(false);
    localization_tune::setDriverDriveLoopTicking(false);
    localization_tune::finalizeInterruptedRunIfNeeded("Interrupted", "Competition init interrupted tune");

    if (kSmokeTestMode) {
        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "SMOKE TEST");
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Mode: Comp Init");
    }
}
