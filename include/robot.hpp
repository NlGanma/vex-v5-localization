#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "main.h"
#include "lemlib/api.hpp"

extern pros::Controller controller;

extern pros::MotorGroup leftDriveMotors;
extern pros::MotorGroup rightDriveMotors;

extern pros::MotorGroup leftPtoMotor1;
extern pros::MotorGroup leftPtoMotor2;
extern pros::MotorGroup rightPtoMotor1;
extern pros::MotorGroup rightPtoMotor2;

extern pros::Imu imu;
extern pros::Distance distFront;
extern pros::Distance distRight;
extern pros::Distance distBack;
extern pros::Distance distLeft;
extern pros::adi::DigitalOut ptoPiston;
extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;
extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;
extern lemlib::Drivetrain drivetrain;

extern lemlib::ControllerSettings fourMotorLinearController;
extern lemlib::ControllerSettings fourMotorAngularController;
extern lemlib::ControllerSettings ptoLinearController;
extern lemlib::ControllerSettings ptoAngularController;

extern lemlib::Chassis chassis;

lemlib::PtoReleaseParams releasedPtoRoles();

#endif
