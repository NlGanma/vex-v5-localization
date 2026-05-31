#include "robot_control.hpp"
#include "pros/rtos.hpp"

#include <array>
#include <atomic>

namespace {
constexpr lemlib::PtoRole kRollerRole = lemlib::PtoRole::MotorRole1;
constexpr lemlib::PtoRole kIntakeRole = lemlib::PtoRole::MotorRole2;

constexpr int kIntakeForwardPower = -127;
constexpr int kIntakeReversePower = 127;
constexpr int kRollerForwardPower = -127;
constexpr int kRollerIdlePower = 35;
constexpr int kRollerReverseFullPower = 127;

constexpr int kPtoShiftDelayMs = 45;
constexpr int kPtoCreepPower4Motor = 127;
constexpr int kPtoCreepPower8Motor = 35;
constexpr int kPtoCreepDelayMs = 55;
constexpr std::uint32_t kShiftWindowTimeoutMs = 300;
constexpr std::uint32_t kFourMotorShiftTimeoutMs = 350;

constexpr int kIntakeBlockSensorPort20ThresholdMm = 50;
constexpr int kIntakeBlockSensorPort5ThresholdMm = 70;
constexpr int kColorSortSlowPower = 100;
constexpr std::uint32_t kColorSortPollIntervalMs = 2;
constexpr std::uint32_t kColorSortRetractDurationMs = 150;
constexpr std::uint32_t kColorSortRearmClearDurationMs = 50;
constexpr std::int32_t kColorSortProximityThreshold = 40;
constexpr std::uint32_t kRightPtoMotor1BlockDelayMs = 350;
constexpr bool kSorterPistonExtendedValue = false;
constexpr bool kSorterPistonRetractedValue = !kSorterPistonExtendedValue;

pros::Distance intakeBlockSensorPort20(20);
pros::Distance intakeBlockSensorPort5(5);
pros::Optical optical16(16);

pros::adi::DigitalOut loadingMechanism('A', false);
pros::adi::DigitalOut middlegoal('C', true);
pros::adi::DigitalOut descore('D', false);
pros::adi::DigitalOut sorterPiston('E', kSorterPistonExtendedValue);

bool loadingMechanismExtended = false;
bool middleGoalExtended = true;
bool descoreExtended = false;
pros::task_t colorSortTaskHandle = nullptr;
std::atomic_bool colorSortMonitoringEnabled {false};
std::atomic_bool colorSortCycleActive {false};
std::atomic_bool colorSortRedLatched {false};
std::atomic<std::uint32_t> colorSortClearStartTimeMs {0};
std::atomic<std::uint32_t> colorSortCycleEndTimeMs {0};
std::atomic<std::uint32_t> rightPtoMotor1BlockStartTimeMs {0};

pros::task_t autonomousManipulatorTaskHandle = nullptr;
std::atomic_bool autonomousIntakeActive {false};
std::atomic<std::uint32_t> autonomousIntakeEndTimeMs {0};
std::atomic_bool eightMotorPositionHoldEnabled {false};

enum class FourMotorShiftState : std::uint8_t { IDLE, FOLLOW, CREEP };
std::atomic<FourMotorShiftState> fourMotorShiftState {FourMotorShiftState::IDLE};

std::array<pros::MotorGroup*, 6> eightMotorHoldGroups() {
    return {&leftDriveMotors, &rightDriveMotors, &leftPtoMotor1, &leftPtoMotor2, &rightPtoMotor1, &rightPtoMotor2};
}

void setAllMotorHoldModes() {
    for (pros::MotorGroup* group : eightMotorHoldGroups()) group->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
}

void commandZeroToAllDriveGroups() {
    for (pros::MotorGroup* group : eightMotorHoldGroups()) group->move(0);
}

void setSorterPiston(bool extended) {
    sorterPiston.set_value(extended ? kSorterPistonExtendedValue : kSorterPistonRetractedValue);
}

void resetColorSortCycle() {
    colorSortCycleActive = false;
    colorSortCycleEndTimeMs = 0;
    setSorterPiston(true);
}

void resetColorSortState() {
    colorSortRedLatched = false;
    colorSortClearStartTimeMs = 0;
    resetColorSortCycle();
}

void resetRightPtoMotor1BlockState() { rightPtoMotor1BlockStartTimeMs = 0; }

bool isRedRingDetected() {
    const double hue = optical16.get_hue();
    return hue > 0 && hue < 20;
}

bool isSorterObjectPresent() {
    const std::int32_t proximity = optical16.get_proximity();
    return proximity != PROS_ERR && proximity > kColorSortProximityThreshold;
}

bool isColorSortCycleActive() { return colorSortCycleActive.load(); }

bool updateColorSortCycle(bool sortEnabled) {
    if (!sortEnabled) {
        if (!colorSortCycleActive.load() &&
            !colorSortRedLatched.load() &&
            colorSortClearStartTimeMs.load() == 0) {
            return false;
        }

        resetColorSortState();
        return false;
    }

    const std::uint32_t now = pros::millis();
    const bool redDetected = isRedRingDetected();
    const bool objectPresent = isSorterObjectPresent();

    if (colorSortCycleActive.load()) {
        if (now < colorSortCycleEndTimeMs.load()) return true;

        resetColorSortCycle();
    }

    if (redDetected && !colorSortRedLatched.load()) {
        colorSortClearStartTimeMs = 0;
        colorSortRedLatched = true;
        colorSortCycleActive = true;
        colorSortCycleEndTimeMs = now + kColorSortRetractDurationMs;
        setSorterPiston(false);
        return true;
    }

    if (!colorSortRedLatched.load()) return false;

    if (objectPresent) {
        colorSortClearStartTimeMs = 0;
        return false;
    }

    const std::uint32_t clearStartTimeMs = colorSortClearStartTimeMs.load();
    if (clearStartTimeMs == 0) {
        colorSortClearStartTimeMs = now;
        return false;
    }

    if (now - clearStartTimeMs >= kColorSortRearmClearDurationMs) {
        colorSortRedLatched = false;
        colorSortClearStartTimeMs = 0;
    }

    return false;
}

void startColorSortControl() {
    if (colorSortTaskHandle != nullptr) return;

    colorSortTaskHandle = pros::Task::create(
        []() {
            while (true) {
                updateColorSortCycle(colorSortMonitoringEnabled.load());
                pros::delay(kColorSortPollIntervalMs);
            }
        },
        "Color Sort");
}

int applyColorSortSlowPower(int power) {
    if (power == 0) return 0;
    return power < 0 ? -kColorSortSlowPower : kColorSortSlowPower;
}

bool isFourMotorShiftActive() { return fourMotorShiftState.load() != FourMotorShiftState::IDLE; }

void commandReleasedPto(int leftRollerPower, int leftIntakePower, int rightIntakePower, int rightIntakePower2) {
    if (chassis.isPtoEngaged() || isFourMotorShiftActive()) return;

    leftPtoMotor1.move(leftRollerPower);
    leftPtoMotor2.move(leftIntakePower);
    rightPtoMotor1.move(rightIntakePower);
    rightPtoMotor2.move(rightIntakePower2);
}

void commandReleasedPto(int intakePower, int rollerPower) {
    commandReleasedPto(rollerPower, intakePower, intakePower, intakePower);
}

void moveAllReleasedPtoGroups(int power) {
    if (chassis.isPtoEngaged()) return;

    leftPtoMotor1.move(-power);
    leftPtoMotor2.move(power);
    rightPtoMotor1.move(power);
    rightPtoMotor2.move(power);
}

void moveReleasedPtoDriveOutputs(float leftPower, float rightPower) {
    if (chassis.isPtoEngaged()) return;

    leftPtoMotor1.move(leftPower);
    leftPtoMotor2.move(leftPower);
    rightPtoMotor1.move(rightPower);
    rightPtoMotor2.move(rightPower);
}

void moveAllReleasedPtoGroupsSameDirection(int power) {
    if (chassis.isPtoEngaged()) return;

    leftPtoMotor1.move(power);
    leftPtoMotor2.move(power);
    rightPtoMotor1.move(power);
    rightPtoMotor2.move(power);
}

void waitForFourMotorShiftComplete() {
    const std::uint32_t startTimeMs = pros::millis();
    while (isFourMotorShiftActive()) {
        if (pros::millis() - startTimeMs >= kFourMotorShiftTimeoutMs) {
            fourMotorShiftState = FourMotorShiftState::IDLE;
            moveAllReleasedPtoGroupsSameDirection(0);
            break;
        }
        pros::delay(10);
    }
}

void startFourMotorShiftTask() {
    pros::Task::create(
        []() {
            const std::uint32_t followEndTimeMs = pros::millis() + kPtoShiftDelayMs;
            while (pros::millis() < followEndTimeMs) {
                moveReleasedPtoDriveOutputs(chassis.getLastCommandedLeftOutput(), chassis.getLastCommandedRightOutput());
                pros::delay(10);
            }

            fourMotorShiftState = FourMotorShiftState::CREEP;
            moveAllReleasedPtoGroupsSameDirection(kPtoCreepPower4Motor);
            pros::delay(kPtoCreepDelayMs);
            moveAllReleasedPtoGroupsSameDirection(0);
            chassis.setControllerSettings(fourMotorLinearController, fourMotorAngularController);
            fourMotorShiftState = FourMotorShiftState::IDLE;
        },
        "4M PTO Shift");
}

bool rightPtoMotor1Blocked() {
    const int sensor20Distance = intakeBlockSensorPort20.get();
    const int sensor5Distance = intakeBlockSensorPort5.get();
    const bool blockedNow = sensor20Distance != PROS_ERR && sensor5Distance != PROS_ERR &&
                            sensor20Distance < kIntakeBlockSensorPort20ThresholdMm &&
                            sensor5Distance < kIntakeBlockSensorPort5ThresholdMm;

    if (!blockedNow) {
        resetRightPtoMotor1BlockState();
        return false;
    }

    const std::uint32_t now = pros::millis();
    const std::uint32_t blockStartTimeMs = rightPtoMotor1BlockStartTimeMs.load();
    if (blockStartTimeMs == 0) {
        rightPtoMotor1BlockStartTimeMs = now;
        return false;
    }

    return now - blockStartTimeMs >= kRightPtoMotor1BlockDelayMs;
}

bool trySwitchToFourMotorDrive() {
    if (isFourMotorShiftActive()) return false;
    if (!chassis.isPtoEngaged()) {
        chassis.disengagePto(releasedPtoRoles(), false);
        chassis.setControllerSettings(fourMotorLinearController, fourMotorAngularController);
        return true;
    }
    if (chassis.isInMotion()) return false;

    stopReleasedPtoControls();
    FourMotorShiftState expectedState = FourMotorShiftState::IDLE;
    if (!fourMotorShiftState.compare_exchange_strong(expectedState, FourMotorShiftState::FOLLOW)) return false;

    chassis.disengagePto(releasedPtoRoles(), false);
    startFourMotorShiftTask();
    return false;
}

bool trySwitchToEightMotorDrive() {
    if (isFourMotorShiftActive()) return false;
    if (chassis.isPtoEngaged()) {
        chassis.setControllerSettings(ptoLinearController, ptoAngularController);
        return true;
    }
    if (chassis.isInMotion()) return false;

    stopReleasedPtoControls();
    chassis.engagePto(false);
    chassis.tank(kPtoCreepPower8Motor, kPtoCreepPower8Motor, true);
    pros::delay(kPtoCreepDelayMs);
    chassis.tank(0, 0, true);
    pros::delay(kPtoShiftDelayMs);
    chassis.setControllerSettings(ptoLinearController, ptoAngularController);
    return true;
}

void waitForShiftWindow() {
    const std::uint32_t startTimeMs = pros::millis();
    while (chassis.isInMotion()) {
        if (pros::millis() - startTimeMs >= kShiftWindowTimeoutMs) {
            chassis.cancelAllMotions();
            chassis.tank(0, 0, true);
            pros::delay(20);
            break;
        }
        pros::delay(10);
    }
}

void startAutonomousManipulatorControl() {
    if (autonomousManipulatorTaskHandle != nullptr) return;

    autonomousManipulatorTaskHandle = pros::Task::create(
        []() {
            while (true) {
                if (!autonomousIntakeActive.load()) {
                    stopReleasedPtoControls();
                    pros::delay(10);
                    continue;
                }

                if (pros::millis() >= autonomousIntakeEndTimeMs.load()) {
                    autonomousIntakeActive = false;
                    stopReleasedPtoControls();
                    pros::delay(10);
                    continue;
                }

                if (trySwitchToFourMotorDrive()) {
                    commandReleasedPto(kIntakeForwardPower, kRollerIdlePower);
                    if (rightPtoMotor1Blocked()) { rightPtoMotor1.brake(); }
                }

                pros::delay(10);
            }
        },
        "Auto Manipulator");
}
} // namespace

void initializeRobotControlState() {
    loadingMechanismExtended = false;
    middleGoalExtended = true;
    descoreExtended = false;
    loadingMechanism.set_value(loadingMechanismExtended);
    middlegoal.set_value(middleGoalExtended);
    descore.set_value(descoreExtended);
    colorSortMonitoringEnabled = false;
    resetColorSortState();
    resetRightPtoMotor1BlockState();
    optical16.set_led_pwm(100);
    startColorSortControl();
    autonomousIntakeActive = false;
    autonomousIntakeEndTimeMs = 0;
    eightMotorPositionHoldEnabled = false;
}

void setAllMotorBrakeModes() {
    leftDriveMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightDriveMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    leftPtoMotor1.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    leftPtoMotor2.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightPtoMotor1.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightPtoMotor2.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
}

void commandTeleopDriveOutputs(int left, int right) {
    leftDriveMotors.move(left);
    rightDriveMotors.move(right);
    if (!chassis.isPtoEngaged()) return;

    leftPtoMotor1.move(left);
    leftPtoMotor2.move(left);
    rightPtoMotor1.move(right);
    rightPtoMotor2.move(right);
}

void enableEightMotorPositionHold() {
    eightMotorPositionHoldEnabled = false;
    chassis.cancelAllMotions();
    stopAutonomousManipulatorControl();

    if (!chassis.isPtoEngaged()) switchToEightMotorDrive();

    setAllMotorHoldModes();
    commandZeroToAllDriveGroups();
    eightMotorPositionHoldEnabled = true;
}

void disableEightMotorPositionHold() {
    eightMotorPositionHoldEnabled = false;
    commandZeroToAllDriveGroups();
    setAllMotorBrakeModes();
}

bool isEightMotorPositionHoldEnabled() { return eightMotorPositionHoldEnabled.load(); }

void stopReleasedPtoControls() {
    colorSortMonitoringEnabled = false;
    resetColorSortState();
    resetRightPtoMotor1BlockState();
    commandReleasedPto(0, 0);
}

void requestSwitchToFourMotorDrive() {
    disableEightMotorPositionHold();
    trySwitchToFourMotorDrive();
}

void requestSwitchToEightMotorDrive() { trySwitchToEightMotorDrive(); }

void switchToFourMotorDrive() {
    disableEightMotorPositionHold();
    waitForShiftWindow();
    requestSwitchToFourMotorDrive();
    waitForFourMotorShiftComplete();
}

void switchToEightMotorDrive() {
    if (isFourMotorShiftActive()) return;
    waitForShiftWindow();
    trySwitchToEightMotorDrive();
}

void setLoadingMechanism(bool extended) {
    loadingMechanismExtended = extended;
    loadingMechanism.set_value(extended);
}

void toggleLoadingMechanism() { setLoadingMechanism(!loadingMechanismExtended); }

void loadingMechanismUp() { setLoadingMechanism(true); }

void loadingMechanismDown() { setLoadingMechanism(false); }

void setMiddleGoal(bool extended) {
    middleGoalExtended = extended;
    middlegoal.set_value(extended);
}

void toggleMiddleGoal() { setMiddleGoal(!middleGoalExtended); }

void middleGoalUp() { setMiddleGoal(true); }

void middleGoalDown() { setMiddleGoal(false); }

void setDescore(bool extended) {
    descoreExtended = extended;
    descore.set_value(extended);
}

void toggleDescore() { setDescore(!descoreExtended); }

void runDriverReleasedPto(bool l1Pressed, bool r1Pressed, bool r2Pressed) {
    if (isFourMotorShiftActive()) {
        colorSortMonitoringEnabled = false;
        return;
    }

    int intakePower = 0;
    int rollerPower = kRollerIdlePower;
    const bool rightPtoMotor1Override = l1Pressed && r2Pressed;
    bool monitorRightPtoMotor1Block = false;

    if (!chassis.isPtoEngaged()) {
        if (l1Pressed) {
            intakePower = kIntakeForwardPower;
            rollerPower = kRollerForwardPower;
        } else if (r1Pressed) {
            intakePower = kIntakeForwardPower;
            rollerPower = kRollerIdlePower;
            monitorRightPtoMotor1Block = true;
        } else if (r2Pressed) {
            intakePower = kIntakeReversePower;
            rollerPower = kRollerReverseFullPower;
        }
    }

    const bool forwardIntakeActive = !chassis.isPtoEngaged() && intakePower == kIntakeForwardPower;
    colorSortMonitoringEnabled = forwardIntakeActive;
    const bool colorSortActiveNow = isColorSortCycleActive();
    const int slowedIntakePower = colorSortActiveNow ? applyColorSortSlowPower(intakePower) : intakePower;

    commandReleasedPto(rollerPower, slowedIntakePower, intakePower, slowedIntakePower);

    if (!monitorRightPtoMotor1Block || rightPtoMotor1Override || chassis.isPtoEngaged()) {
        resetRightPtoMotor1BlockState();
    }

    if (monitorRightPtoMotor1Block && !rightPtoMotor1Override && !chassis.isPtoEngaged() && rightPtoMotor1Blocked()) {
        rightPtoMotor1.brake();
    }
}

void intake(std::uint32_t durationMs) {
    disableEightMotorPositionHold();

    if (durationMs == 0) {
        autonomousIntakeActive = false;
        stopReleasedPtoControls();
        return;
    }

    startAutonomousManipulatorControl();
    autonomousIntakeEndTimeMs = pros::millis() + durationMs;
    autonomousIntakeActive = true;
}

void stopAutonomousManipulatorControl() {
    autonomousIntakeActive = false;
    autonomousIntakeEndTimeMs = 0;

    // Remove the task FIRST, then zero. Zeroing before the task is gone left a
    // window where the task (having already read autonomousIntakeActive==true)
    // could re-issue intake/roller power AFTER the zero and then be frozen by
    // task.remove() with the released PTO motors still running -- and nothing on
    // the disabled()/opcontrol-entry path re-zeros those motors.
    if (autonomousManipulatorTaskHandle != nullptr) {
        pros::Task task(autonomousManipulatorTaskHandle);
        task.remove();
        autonomousManipulatorTaskHandle = nullptr;
    }

    stopReleasedPtoControls();
}

void score(std::uint32_t durationMs, int direction) {
    const int normalizedDirection = direction >= 0 ? 1 : -1;
    const int scorePower = -127 * normalizedDirection;
    const std::uint32_t scoreEndTimeMs = pros::millis() + durationMs;

    disableEightMotorPositionHold();
    stopAutonomousManipulatorControl();
    switchToFourMotorDrive();

    while (pros::millis() < scoreEndTimeMs) {
        commandReleasedPto(scorePower, scorePower);
        pros::delay(10);
    }

    stopReleasedPtoControls();
    switchToEightMotorDrive();
}
