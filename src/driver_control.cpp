#include "main.h"

#include "app_config.hpp"
#include "localization_tune.hpp"
#include "robot_control.hpp"

#include <algorithm>

namespace {
void stopChassisMotion() {
    chassis.cancelAllMotions();
    chassis.tank(0, 0, true);
}
} // namespace

void opcontrol() {
    localization_tune::setDriverControlLoopActive(true);
    localization_tune::setDriverDriveLoopTicking(false);
    localization_tune::finalizeInterruptedRunIfNeeded("Interrupted", "Entered driver control");
    stopChassisMotion();
    // Take back a motion left owned by a killed autonomous task, and settle
    // correction suppression (skipped autonomous->driver transitions bypass disabled()).
    chassis.recoverInterruptedMotion();
    // The boundary re-anchor one-shot is between-motions-while-stopped only. In
    // teleop no driver motion re-suppresses corrections, so an armed request
    // would commit a bounded pose step while the driver is moving. Drop any
    // request left from autonomous here, and again after every 8-motor engage:
    // its creep is a chassis motion (drivePulse) whose endMotion re-arms it, and
    // any legal stopped commit has already had its window by then.
    lemlib::localization::clearBoundaryReanchor();

    if (kSmokeTestMode) {
        bool rumbled = false;
        showSmokeTestStatus("SMOKE TEST", "Mode: Driver");
        while (true) {
            if (!rumbled) {
                controller.rumble(".");
                rumbled = true;
            }

            const int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            const int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            const int left = std::clamp(throttle + turn, -127, 127);
            const int right = std::clamp(throttle - turn, -127, 127);

            leftDriveMotors.move(left);
            rightDriveMotors.move(right);

            showSmokeTestDrive(left, right);
            pros::delay(20);
        }
    }

    stopAutonomousManipulatorControl();
    setAllMotorBrakeModes();
    switchToEightMotorDrive();
    lemlib::localization::clearBoundaryReanchor();
    // Start teleop in a known retracted state. Using toggleMiddleGoal() here made
    // the starting position depend on the prior path into opcontrol (it only
    // lands "down" if it happened to be "up" coming in).
    middleGoalDown();

    // VEXos drops controller text writes closer than 10 ms (wired) / 50 ms
    // (VEXnet) apart, so issue one write per slot: blank lines 1 and 2 once, then
    // alternate the pose and status lines (each still refreshes every 200 ms).
    // A rejected write is retried in the next slot. Blanks use set_text, not
    // clear_line: clear_line returns VEXos's raw result (0 when rate-rejected,
    // not PROS_ERR), so a dropped clear would be indistinguishable from success.
    constexpr std::uint32_t kControllerTextSlotMs = 100;
    constexpr char kBlankControllerLine[] = "                   "; // full 19-column line
    static_assert(sizeof(kBlankControllerLine) - 1 == 19);
    std::uint32_t nextControllerDisplayUpdate = pros::millis() + kControllerTextSlotMs;
    int controllerTextStep = 0; // 0: blank line 1, 1: blank line 2, then 2 (pose) / 3 (CTL)
    bool upPressedLastCycle = false;
    bool r1PressedLastCycle = false;
    bool r2PressedLastCycle = false;
    bool effectiveL1PressedLastCycle = false;
    bool autoReturnToEightMotorDrivePending = false;

    while (true) {
        localization_tune::setDriverDriveLoopTicking(true);

        if (pros::millis() >= nextControllerDisplayUpdate) {
            std::int32_t written = PROS_ERR;
            if (controllerTextStep == 0) {
                written = controller.set_text(1, 0, kBlankControllerLine);
            } else if (controllerTextStep == 1) {
                written = controller.set_text(2, 0, kBlankControllerLine);
            } else if (controllerTextStep == 2) {
                const lemlib::Pose currentPose = chassis.getPose();
                written = controller.print(0, 0, "X%3.0f Y%3.0f T%3.0f",
                                           static_cast<double>(currentPose.x),
                                           static_cast<double>(currentPose.y),
                                           static_cast<double>(currentPose.theta));
            } else {
                const int controllerConnected = pros::c::controller_is_connected(pros::E_CONTROLLER_MASTER);
                written = controller.print(1, 0, "CTL %d", controllerConnected);
            }
            if (written != PROS_ERR) controllerTextStep = controllerTextStep < 3 ? controllerTextStep + 1 : 2;
            nextControllerDisplayUpdate = pros::millis() + kControllerTextSlotMs;
        }

        if (pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_X)) {
            switchToEightMotorDrive();
            lemlib::localization::clearBoundaryReanchor();
        }
        if (pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_Y))
            toggleLoadingMechanism();
        if (pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_L2))
            toggleDescore();

        const int leftY = pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int rightX =
            pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X);
        const int leftDrive = std::clamp(leftY + rightX, -127, 127);
        const int rightDrive = std::clamp(leftY - rightX, -127, 127);
        commandTeleopDriveOutputs(leftDrive, rightDrive);

        const bool l1Pressed = pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_L1);
        const bool upPressed = pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_UP);
        const bool r1Pressed = pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_R1);
        const bool r2Pressed = pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_R2);
        const bool effectiveL1Pressed = l1Pressed || upPressed;

        if ((!r1Pressed && r1PressedLastCycle) ||
            (!r2Pressed && r2PressedLastCycle) ||
            (!effectiveL1Pressed && effectiveL1PressedLastCycle)) {
            autoReturnToEightMotorDrivePending = true;
        }

        if (upPressed && !upPressedLastCycle) middleGoalUp();

        const bool wantsIntake = effectiveL1Pressed || r1Pressed || r2Pressed;
        if (wantsIntake && chassis.isPtoEngaged()) requestSwitchToFourMotorDrive();

        if (autoReturnToEightMotorDrivePending && !effectiveL1Pressed && !r1Pressed && !r2Pressed) {
            requestSwitchToEightMotorDrive();
            lemlib::localization::clearBoundaryReanchor();
            autoReturnToEightMotorDrivePending = !chassis.isPtoEngaged();
        }

        if (!upPressed && upPressedLastCycle) {
            stopReleasedPtoControls();
            middleGoalDown();
        }

        runDriverReleasedPto(effectiveL1Pressed, r1Pressed, r2Pressed);
        upPressedLastCycle = upPressed;
        r1PressedLastCycle = r1Pressed;
        r2PressedLastCycle = r2Pressed;
        effectiveL1PressedLastCycle = effectiveL1Pressed;
        pros::delay(10);
    }
}
