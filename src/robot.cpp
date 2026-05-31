#include "robot.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftDriveMotors({-4, -2}, pros::MotorGearset::blue);
pros::MotorGroup rightDriveMotors({9, 7}, pros::MotorGearset::blue);

pros::MotorGroup leftPtoMotor1({1}, pros::MotorGearset::blue);
pros::MotorGroup leftPtoMotor2({3}, pros::MotorGearset::blue);
pros::MotorGroup rightPtoMotor1({-10}, pros::MotorGearset::blue);
pros::MotorGroup rightPtoMotor2({-8}, pros::MotorGearset::blue);

pros::Imu imu(18);
pros::Distance distFront(13);
pros::Distance distRight(17);
pros::Distance distBack(12);
pros::Distance distLeft(15);
pros::adi::DigitalOut ptoPiston('B', true);

pros::Rotation horizontalEnc(14);
pros::Rotation verticalEnc(19);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -6.75);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.125);

lemlib::Drivetrain drivetrain(&leftDriveMotors,
                              &rightDriveMotors,
                              11.375,
                              lemlib::Omniwheel::NEW_325,
                              450,
                              8);

// 4-motor controller profile.
lemlib::ControllerSettings fourMotorLinearController(13.2,
                                                     0,
                                                     129,
                                                     0,
                                                     1,
                                                     100,
                                                     2,
                                                     500,
                                                     0);

lemlib::ControllerSettings fourMotorAngularController(4.8,
                                                      0,
                                                      37,
                                                      0,
                                                      1,
                                                      100,
                                                      3,
                                                      500,
                                                      40);

// 8-motor PTO-drive profile.
lemlib::ControllerSettings ptoLinearController(13.2,
                                               0,
                                               105,
                                               0,
                                               1,
                                               100,
                                               2,
                                               500,
                                               0);

lemlib::ControllerSettings ptoAngularController(4.73,
                                                0,
                                                50,
                                                0,
                                                1,
                                                100,
                                                3,
                                                500,
                                                40);

namespace {
lemlib::OdomSensors sensors(&vertical,
                            nullptr,
                            &horizontal,
                            nullptr,
                            &imu);

lemlib::ExpoDriveCurve throttleCurve(3,
                                     10,
                                     1.019);

lemlib::ExpoDriveCurve steerCurve(3,
                                  10,
                                  1.019);
} // namespace

lemlib::Chassis chassis(
    drivetrain, fourMotorLinearController, fourMotorAngularController, sensors, &throttleCurve, &steerCurve);

lemlib::PtoReleaseParams releasedPtoRoles() {
    lemlib::PtoReleaseParams params;
    params.motorRoles[0] = lemlib::PtoRole::MotorRole1;
    params.motorRoles[1] = lemlib::PtoRole::MotorRole2;
    params.motorRoles[2] = lemlib::PtoRole::MotorRole2;
    params.motorRoles[3] = lemlib::PtoRole::MotorRole2;
    return params;
}
