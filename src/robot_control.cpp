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

// The autonomous manipulator is lock-free: its start/stop paths run on PROS
// competition tasks, which are deleted on every mode change without releasing
// any mutex they own. Each task serves exactly one generation.
std::atomic<std::uint32_t> autonomousManipulatorGeneration {0};
std::atomic<std::uint32_t> autonomousManipulatorTasksAlive {0};
std::atomic_bool eightMotorPositionHoldEnabled {false};

enum class FourMotorShiftState : std::uint8_t { IDLE, FOLLOW, CREEP };
std::atomic<FourMotorShiftState> fourMotorShiftState {FourMotorShiftState::IDLE};
std::atomic_bool eightMotorControllerProfileActive {false};
std::atomic<std::uint32_t> fourMotorShiftGeneration {0};
// Competition tasks hold this only around atomics (never a delay or a kernel task
// create/delete), so a mode-change kill cannot orphan it across a long window.
pros::Mutex fourMotorShiftLifecycleMutex;

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

void hardStopReleasedPtoGroups() {
    // Engaged PTO motors are drive motors owned by moveDrive/engage/disengage.
    if (chassis.isPtoEngaged()) return;

    leftPtoMotor1.move(0);
    leftPtoMotor2.move(0);
    rightPtoMotor1.move(0);
    rightPtoMotor2.move(0);
}

void cancelFourMotorShiftTask() {
    // The shift task is never deleted: it re-checks the generation under this lock
    // before every write, so it leaves on its own and cannot overwrite the stop.
    fourMotorShiftLifecycleMutex.take();
    fourMotorShiftGeneration.fetch_add(1);
    fourMotorShiftState.store(FourMotorShiftState::IDLE);
    fourMotorShiftLifecycleMutex.give();
    hardStopReleasedPtoGroups();
}

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
            cancelFourMotorShiftTask();
            break;
        }
        pros::delay(10);
    }
}

void startFourMotorShiftTask() {
    fourMotorShiftLifecycleMutex.take();
    if (fourMotorShiftState.load() != FourMotorShiftState::FOLLOW) {
        fourMotorShiftLifecycleMutex.give();
        return;
    }
    const std::uint32_t generation = fourMotorShiftGeneration.fetch_add(1) + 1;
    fourMotorShiftLifecycleMutex.give();

    // Created outside the lock. A cancel that lands before this task first runs
    // bumps the generation, so the task returns without a single write.
    pros::Task::create(
        [generation]() {
            const std::uint32_t followEndTimeMs = pros::millis() + kPtoShiftDelayMs;
            while (fourMotorShiftGeneration.load() == generation && pros::millis() < followEndTimeMs) {
                fourMotorShiftLifecycleMutex.take();
                if (fourMotorShiftGeneration.load() != generation) {
                    fourMotorShiftLifecycleMutex.give();
                    return;
                }
                moveReleasedPtoDriveOutputs(chassis.getLastCommandedLeftOutput(), chassis.getLastCommandedRightOutput());
                fourMotorShiftLifecycleMutex.give();
                pros::delay(10);
            }

            fourMotorShiftLifecycleMutex.take();
            if (fourMotorShiftGeneration.load() != generation) {
                fourMotorShiftLifecycleMutex.give();
                return;
            }
            fourMotorShiftState = FourMotorShiftState::CREEP;
            moveAllReleasedPtoGroupsSameDirection(kPtoCreepPower4Motor);
            fourMotorShiftLifecycleMutex.give();
            pros::delay(kPtoCreepDelayMs);

            fourMotorShiftLifecycleMutex.take();
            if (fourMotorShiftGeneration.load() != generation) {
                fourMotorShiftLifecycleMutex.give();
                return;
            }
            moveAllReleasedPtoGroupsSameDirection(0);
            // Never retune (and reset PID/exit state) under a running motion; the
            // released branch of trySwitchToFourMotorDrive applies it at a boundary.
            if (!chassis.isInMotion()) {
                chassis.setControllerSettings(fourMotorLinearController, fourMotorAngularController);
                eightMotorControllerProfileActive.store(false);
            }
            fourMotorShiftState = FourMotorShiftState::IDLE;
            fourMotorShiftLifecycleMutex.give();
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
        // Mechanism permission follows the physical PTO state; a deferred or
        // aborted-shift 4-motor gain swap waits for a motion-free call.
        if (eightMotorControllerProfileActive.load() && !chassis.isInMotion()) {
            chassis.setControllerSettings(fourMotorLinearController, fourMotorAngularController);
            eightMotorControllerProfileActive.store(false);
        }
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
        if (eightMotorControllerProfileActive.load()) return true;
        if (chassis.isInMotion()) return false;
        chassis.setControllerSettings(ptoLinearController, ptoAngularController);
        eightMotorControllerProfileActive.store(true);
        return true;
    }
    if (chassis.isInMotion()) return false;

    stopReleasedPtoControls();
    chassis.engagePto(false);
    chassis.drivePulse(kPtoCreepPower8Motor, kPtoCreepDelayMs, false);
    pros::delay(kPtoShiftDelayMs);
    chassis.setControllerSettings(ptoLinearController, ptoAngularController);
    eightMotorControllerProfileActive.store(true);
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

void startAutonomousManipulatorTask(std::uint32_t generation, std::uint32_t endTimeMs) {
    pros::Task::create(
        [generation, endTimeMs]() {
            // Count in before the first generation check: a caller that bumps the
            // generation and then reads zero here is guaranteed this task sees the
            // new generation and leaves without commanding anything.
            autonomousManipulatorTasksAlive.fetch_add(1);
            while (autonomousManipulatorGeneration.load() == generation &&
                   !pros::competition::is_disabled()) {
                if (pros::millis() >= endTimeMs) {
                    stopReleasedPtoControls();
                    break;
                }

                if (trySwitchToFourMotorDrive()) {
                    commandReleasedPto(kIntakeForwardPower, kRollerIdlePower);
                    if (rightPtoMotor1Blocked()) { rightPtoMotor1.brake(); }
                }

                pros::delay(10);
            }
            autonomousManipulatorTasksAlive.fetch_sub(1);
        },
        "Auto Manipulator");
}

// Call after bumping autonomousManipulatorGeneration. Let any older task leave
// its PTO helpers cooperatively instead of deleting it while it might own the
// shift mutex. Its loop and every PTO wait are bounded; normally this is a single
// 10 ms scheduler tick.
void waitForAutonomousManipulatorExit() {
    const std::uint32_t waitStartMs = pros::millis();
    while (autonomousManipulatorTasksAlive.load() != 0 && pros::millis() - waitStartMs < 100) {
        pros::delay(5);
    }
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
    eightMotorPositionHoldEnabled = false;
    eightMotorControllerProfileActive = false;
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
    // keep the chassis last-commanded outputs live: the 4-motor PTO shift's
    // FOLLOW phase reads them, and without this a teleop mid-drive shift would
    // command the still-meshed PTO motors 0 for the disengage window
    chassis.noteExternalDriveCommand(static_cast<float>(left), static_cast<float>(right));
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
    if (!eightMotorPositionHoldEnabled.exchange(false)) return;
    commandZeroToAllDriveGroups();
    setAllMotorBrakeModes();
}

bool isEightMotorPositionHoldEnabled() { return eightMotorPositionHoldEnabled.load(); }

void stopReleasedPtoControls() {
    cancelFourMotorShiftTask();
    colorSortMonitoringEnabled = false;
    resetColorSortState();
    resetRightPtoMotor1BlockState();
    hardStopReleasedPtoGroups();
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
    if (isFourMotorShiftActive()) waitForFourMotorShiftComplete();
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
        stopAutonomousManipulatorControl();
        return;
    }
    if (pros::competition::is_disabled()) return;

    // Retire any running task (it may already be on its expiry exit path) and
    // start a fresh one, so this request can never be swallowed by that exit.
    const std::uint32_t generation = autonomousManipulatorGeneration.fetch_add(1) + 1;
    waitForAutonomousManipulatorExit();
    startAutonomousManipulatorTask(generation, pros::millis() + durationMs);
}

void stopAutonomousManipulatorControl() {
    autonomousManipulatorGeneration.fetch_add(1);
    waitForAutonomousManipulatorExit();

    // Zero only after the generation has been invalidated and the task has had
    // a chance to leave, so it cannot re-issue a stale mechanism command later.
    stopReleasedPtoControls();
}

void score(std::uint32_t durationMs, int direction) {
    const int normalizedDirection = direction >= 0 ? 1 : -1;
    const int scorePower = -127 * normalizedDirection;
    disableEightMotorPositionHold();
    stopAutonomousManipulatorControl();
    switchToFourMotorDrive();

    const std::uint32_t scoreEndTimeMs = pros::millis() + durationMs;
    while (pros::millis() < scoreEndTimeMs) {
        commandReleasedPto(scorePower, scorePower);
        pros::delay(10);
    }

    stopReleasedPtoControls();
    // Air pump/PTO is unavailable for validation runs, so leave the robot in
    // released 4-motor mode after scoring instead of trying to shift back to
    // 8-motor drive.
    switchToFourMotorDrive();
}
