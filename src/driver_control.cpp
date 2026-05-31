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

    if (kSmokeTestMode) {
        bool rumbled = false;
        while (true) {
            if (!rumbled) {
                controller.rumble(".");
                rumbled = true;
            }
            pros::screen::erase();
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "SMOKE TEST");
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Mode: Driver");

            const int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            const int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            const int left = std::clamp(throttle + turn, -127, 127);
            const int right = std::clamp(throttle - turn, -127, 127);

            leftDriveMotors.move(left);
            rightDriveMotors.move(right);

            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "L %d R %d", left, right);
            pros::delay(20);
        }
    }

    stopAutonomousManipulatorControl();
    setAllMotorBrakeModes();
    switchToEightMotorDrive();
    // Start teleop in a known retracted state. Using toggleMiddleGoal() here made
    // the starting position depend on the prior path into opcontrol (it only
    // lands "down" if it happened to be "up" coming in).
    middleGoalDown();
    controller.clear_line(1);
    controller.clear_line(2);

    std::uint32_t nextControllerDisplayUpdate = 0;
    bool upPressedLastCycle = false;
    bool r1PressedLastCycle = false;
    bool r2PressedLastCycle = false;
    bool effectiveL1PressedLastCycle = false;
    bool autoReturnToEightMotorDrivePending = false;

    while (true) {
        localization_tune::setDriverDriveLoopTicking(true);

        if (pros::millis() >= nextControllerDisplayUpdate) {
            const lemlib::Pose currentPose = chassis.getPose();
            const int controllerConnected = pros::c::controller_is_connected(pros::E_CONTROLLER_MASTER);
            controller.print(0, 0, "X%3.0f Y%3.0f T%3.0f",
                             static_cast<double>(currentPose.x),
                             static_cast<double>(currentPose.y),
                             static_cast<double>(currentPose.theta));
            controller.print(1, 0, "CTL %d", controllerConnected);
            nextControllerDisplayUpdate = pros::millis() + 200;
        }

        if (pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_X))
            switchToEightMotorDrive();
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
