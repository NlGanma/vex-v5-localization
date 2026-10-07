#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

#include "robot.hpp"

#include <cstdint>

void initializeRobotControlState();
void setAllMotorBrakeModes();
void commandTeleopDriveOutputs(int left, int right);
void stopReleasedPtoControls();
void enableEightMotorPositionHold();
void disableEightMotorPositionHold();
bool isEightMotorPositionHoldEnabled();

void requestSwitchToFourMotorDrive();
void requestSwitchToEightMotorDrive();
void switchToFourMotorDrive();
void switchToEightMotorDrive();

void setLoadingMechanism(bool extended);
void toggleLoadingMechanism();
void loadingMechanismUp();
void loadingMechanismDown();

void setMiddleGoal(bool extended);
void toggleMiddleGoal();
void middleGoalUp();
void middleGoalDown();

void setDescore(bool extended);
void toggleDescore();

void runDriverReleasedPto(bool l1Pressed, bool r1Pressed, bool r2Pressed);

void intake(std::uint32_t durationMs);
void stopAutonomousManipulatorControl();
void score(std::uint32_t durationMs, int direction);

// Smoke-test mode screen text. Only the persistent display task calls pros::screen:
// the kernel screen mutex is outside the daemon's port-mutex barrier, so a
// competition task deleted mid-print would orphan it.
void startSmokeTestDisplay();
void showSmokeTestStatus(const char* line1, const char* line2);
void showSmokeTestDrive(int left, int right);

#endif
