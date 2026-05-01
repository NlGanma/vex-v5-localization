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

#endif
