#include "main.h"

#include "app_config.hpp"
#include "autonomous_localization.hpp"
#include "lemlib/localization/localization.hpp"
#include "localization_tune.hpp"
#include "robot_control.hpp"

namespace {
// Normal competition autonomous is authored from a zeroed local frame:
// local (0, 0, 0) = robot placed at the start
// local +Y = forward from the starting heading
// local +X = right from the starting heading
// local heading 0 deg = the starting heading
//
// The localization runtime still stays in absolute field coordinates underneath,
// so MCL + EKF + odometry remain active during autonomous. Update these three
// numbers to the real absolute field pose for this routine's start.
constexpr float kAutonomousStartAbsX = -27.0f;
constexpr float kAutonomousStartAbsY = -36.0f;
constexpr float kAutonomousStartHeadingDeg = 0.0f;
constexpr int kLocalizationTuneTest = 0; // 0 normal route, 1 turn/center, 2 straight scale, 3 square loop, 4 drive probe, 5 sensor angle, 6 square + cross

void stopChassisMotion() {
    chassis.cancelAllMotions();
    chassis.tank(0, 0, true);
}

bool prepareAutonomousStart(autonomous_localization::StartRelativeChassis* localChassis) {
    disableEightMotorPositionHold();
    stopAutonomousManipulatorControl();
    stopReleasedPtoControls();
    setAllMotorBrakeModes();
    stopChassisMotion();
    switchToFourMotorDrive();
    loadingMechanismDown();
    middleGoalUp();
    setDescore(false);

    const lemlib::Pose absoluteStartPose(
        kAutonomousStartAbsX, kAutonomousStartAbsY, lemlib::degToRad(kAutonomousStartHeadingDeg));
    if (!autonomous_localization::beginAutonomousWithFixedStart(absoluteStartPose, localChassis)) {
        stopChassisMotion();
        return false;
    }

    return true;
}
} // namespace

void autonomous() {
    if (kSmokeTestMode) {
        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "SMOKE TEST");
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Mode: Autonomous");
        leftDriveMotors.move(80);
        rightDriveMotors.move(80);
        pros::delay(1000);
        leftDriveMotors.move(0);
        rightDriveMotors.move(0);
        pros::delay(500);
        leftDriveMotors.move(-80);
        rightDriveMotors.move(-80);
        pros::delay(1000);
        leftDriveMotors.move(0);
        rightDriveMotors.move(0);
        return;
    }

    autonomous_localization::StartRelativeChassis localChassis {};
    if (!prepareAutonomousStart(&localChassis)) return;
    if (kLocalizationTuneTest >= 1 && kLocalizationTuneTest <= 6) {
        const lemlib::Pose tuneStart(kAutonomousStartAbsX, kAutonomousStartAbsY,
                                     lemlib::degToRad(kAutonomousStartHeadingDeg));
        localization_tune::runAutonomousRoute(tuneStart, kLocalizationTuneTest, true);
        return;
    }

    auto& chassis = localChassis;

    // Anchor the local route frame to the field. While stationary at the start,
    // relocalize against the perimeter walls: on a confident solve, local
    // (0, 0, 0) becomes the true field pose so localization corrects the run; if
    // relocalization fails, fall back to the configured fixed start so the robot
    // runs the identical start-relative route on odometry from local (0, 0, 0)
    // while normal gated range fusion stays tied to that track. The physical
    // route is the same either way. (Tune tests above relocalize inside
    // runAutonomousRoute, so this only runs for the normal route.)
    const lemlib::Pose autonomousFixedStart(kAutonomousStartAbsX, kAutonomousStartAbsY,
                                            lemlib::degToRad(kAutonomousStartHeadingDeg));
    localization_tune::prepareAutonomousRelocalizedRun(autonomousFixedStart);
    localization_tune::setState("Setup", "Relocalize", true);
    lemlib::localization::setTraceEnabled(true);
    autonomous_localization::RelocalizationSummary startRelocalization {};
    autonomous_localization::beginAutonomousWithRelocalizationOrFixedStart(autonomousFixedStart, &localChassis,
                                                                           &startRelocalization);
    localization_tune::storeRelocalizationSummary(startRelocalization);
    localization_tune::setStartPose(startRelocalization.success ? startRelocalization.pose : autonomousFixedStart);

    // Route commands below are start-relative, not field-center-relative.
    // The internal fused pose is still absolute, but this local wrapper lets
    // you author autonomous like the robot was zeroed at the start tile.
    //
    // If you ever need a raw absolute-field command for debugging, call
    // ::chassis directly instead of this local alias.
    // Pose helpers:
    // const lemlib::Pose poseDeg = chassis.getPose();                     // Read the current local pose, heading in degrees.
    // const lemlib::Pose poseRad = chassis.getPose(true);                 // Read the current local pose, heading in radians.
    // chassis.setPose(0.0f, 0.0f, 0.0f);                                 // Re-zero the local pose at the current start frame.
    // chassis.setPose(lemlib::Pose(10.0f, 20.0f, 1.57f), true);          // Override the local pose using radians.

    // Start-relative chassis motion commands:
    // chassis.moveToPoint(0.0f, 10.0f, 1500);                            // Move 10 inches forward from the start.
    // chassis.moveToPoint(10.0f, 0.0f, 1500);                            // Move 10 inches right from the start.
    // chassis.moveToPoint(40.0f, 5.0f, 2000, {.forwards = false});       // Drive backwards to the local point (40, 5).
    // chassis.moveToPose(30.0f, 50.0f, 180.0f, 2500);                    // Drive to a local point and finish 180 deg from start heading.
    // chassis.turnToHeading(90.0f, 1200);                                // Turn in place to 90 deg relative to the start heading.
    // chassis.turnToPoint(24.0f, 24.0f, 1200);                           // Turn to face the local point (24, 24).
    // chassis.swingToHeading(180.0f, lemlib::DriveSide::LEFT, 1200);     // Swing turn while locking the left side.
    // chassis.swingToPoint(0.0f, 48.0f, lemlib::DriveSide::RIGHT, 1200); // Swing turn while locking the right side.
    // ::chassis.follow(my_path_txt, 12.0f, 3000, true, false);           // Path assets stay in their authored absolute frame.

    // Chassis motion flow helpers:
    // chassis.waitUntil(10.0f);                                          // Wait until the current motion is within 10 in/deg.
    // chassis.waitUntilDone();                                           // Wait until the current motion fully completes.
    // chassis.cancelMotion();                                            // Cancel only the current active motion.
    // chassis.cancelAllMotions();                                        // Cancel the current motion and anything queued.

    // PTO / mechanism wrappers for this robot:
    // switchToFourMotorDrive();                                          // Release PTO so the extra motors become intake/roller.
    // switchToEightMotorDrive();                                         // Re-engage PTO so all eight motors drive the chassis.
    // requestSwitchToFourMotorDrive();                                   // Non-blocking request to switch to 4-motor mode.
    // requestSwitchToEightMotorDrive();                                  // Non-blocking request to switch to 8-motor mode.
    // intake(700);                                                       // Run the autonomous intake helper for 700 ms.
    // intake(0);                                                         // Stop the autonomous intake helper immediately.
    // score(300, 1);                                                     // Score forwards for 300 ms.
    // score(300, -1);                                                    // Score in reverse for 300 ms.
    // stopAutonomousManipulatorControl();                                // Hard-stop any autonomous intake helper task.
    // loadingMechanismUp();                                              // Raise the loading mechanism.
    // loadingMechanismDown();                                            // Lower the loading mechanism.
    // toggleLoadingMechanism();                                          // Toggle the loading mechanism.
    // setLoadingMechanism(true);                                         // Explicitly set the loading mechanism extended.
    // middleGoalUp();                                                    // Raise the middle-goal mechanism.
    // middleGoalDown();                                                  // Lower the middle-goal mechanism.
    // toggleMiddleGoal();                                                // Toggle the middle-goal mechanism.
    // setMiddleGoal(true);                                               // Explicitly set the middle-goal mechanism extended.
    // setDescore(true);                                                  // Extend the descore mechanism.
    // setDescore(false);                                                 // Retract the descore mechanism.
    // toggleDescore();                                                   // Toggle the descore mechanism.

    // Active route: 4-motor validation autonomous. All motion calls are
    // blocking so the trace exercises the intended sequence deterministically.
    // Myron is the goat.
    middleGoalDown();
    // This is localChassis, not ::chassis: local (0, 0, 0) is wherever the robot
    // is physically placed at autonomous start, not the field center.
    chassis.setPose(0, 0, 0);
    intake(90000);
    chassis.moveToPoint(5.5, 16, 1500, {.maxSpeed = 127, .minSpeed = 40}, false);
    loadingMechanismDown();
    chassis.moveToPoint(28.8, -1.2, 1500, {.maxSpeed = 127, .minSpeed = 60}, false);
    chassis.turnToHeading(183, 300, {.maxSpeed = 127, .minSpeed = 80}, false);
    chassis.drivePulse(66, 675);
    chassis.moveToPoint(30.1, 0, 1500, {.forwards = false, .maxSpeed = 127, .minSpeed = 6}, false);
    loadingMechanismUp();
    chassis.turnToHeading(-45, 500, {.maxSpeed = 127, .minSpeed = 100}, false);
    chassis.turnToHeading(2, 700, {.maxSpeed = 100, .minSpeed = 10}, false);
    chassis.drivePulse(100, 410);
    score(1250, 127);
    chassis.drivePulse(-60, 300);
    chassis.turnToHeading(41, 500, {.maxSpeed = 127, .minSpeed = 50}, false);
    chassis.drivePulse(70, 345);
    chassis.moveToPoint(36.5, 55, 1000, {.maxSpeed = 127, .minSpeed = 90}, false);
    chassis.turnToHeading(30, 500, {.maxSpeed = 85, .minSpeed = 85}, false);
    localization_tune::finalizeRun("Complete", "4-motor validation route", "--");
}
