#include "main.h"
#include "Customfn/block_detection.hpp"
#include "dismove.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/forward.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/motors.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include <cmath>
#include "Customfn/block_detection.hpp"
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2, -3, -1}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({10, 9, 8}, pros::MotorGearset::blue); 

void left_motors(pros::motor_brake_mode_e_t mode) {
    for (int p : {-2, -3, -1}) pros::Motor(std::abs(p)).set_brake_mode(mode);
}
void right_motors(pros::motor_brake_mode_e_t mode) {
    for (int p : {10, 9, 8}) pros::Motor(std::abs(p)).set_brake_mode(mode);
}
// Inertial Sensor on port 10
pros::Imu imu(12);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(21); //TBD
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-14);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.7);

// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -0.8);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.6, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              4.5 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            60, // derivative gain (kD)
                                            0, // anti windup
                                            0.5, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             50, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
//  motors
pros::Motor intake(7); // INTAKE
pros::MotorGroup push({20, -11}, pros::MotorGearset::green);

//  pneumatics 
pros::adi::Pneumatics loading('A', false); // loading
pros::adi::Pneumatics dis('B', false);     // dis
pros::adi::Pneumatics colors('C', false);  // colors
pros::adi::Pneumatics parking('D', false); // parking

// sensors
pros::Distance dist_push_start(6);  // Distancepushstart
pros::Distance dist_end(4);         // Distanceend
pros::Distance dist_f(5);          // DisF
pros::Distance dist_back(13);       // Disback    
pros::Optical optical16(16);        // Optical16

//instance
BlockDetection blockDetection;

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(pathtomidgoal_jerryio_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 */
void autonomous(){
    left_motors(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors(pros::E_MOTOR_BRAKE_BRAKE);
    intake.move(127);
    parking.retract();
    chassis.setPose(0,0,0);
    imu.set_heading(0);
    chassis.moveToPoint(0.05, 19, 100000, {.minSpeed = 25});
    chassis.moveToPoint(0.08, 32.5, 100000, {.maxSpeed = 20});
    chassis.turnToHeading(-95, 1000);
    chassis.moveToPoint(12, 24, 650, {.forwards = false, .maxSpeed = 50, .minSpeed = 0}, false);
    pros::delay(50);
    colors.extend();
    push.move(-30);
    pros::delay(1000);
    intake.move(127);
    chassis.moveToPoint(-38.5, 12, 2500, {.minSpeed = 35},  false);
    push.move(0);
    colors.retract();
    chassis.swingToHeading(-155, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 35}, false);
    loading.extend();
    blockDetection.start(&push, &intake, &dist_push_start, &dist_end);
    dismove_dis_maxspeed_max_min_min_kp_kp(50000, 100, 65, 0.8); //in the first loader
    pros::delay(1200);
    dismove_dis_maxspeed_max_min_min_kp_kp(-45000, 100, 5, 0.8);
    loading.retract();
    leftMotors.move(0);
    rightMotors.move(0);
    chassis.cancelMotion();
    pros::delay(20);
    chassis.turnToHeading(30, 10000, {.maxSpeed = 85, .minSpeed = 10, .earlyExitRange = 1}, false);
    dismove_dis_maxspeed_max_min_min_kp_kp(40000, 70, 10, 0.8);
    blockDetection.stop();
    intake.move(127);
    push.move(127);
    pros::delay(700);
    dismove_dis_maxspeed_max_min_min_kp_kp(-10000, 70, 10, 0.8);
    colors.extend();
    push.move(-127);
    chassis.turnToHeading(110, 500);
    chassis.moveToPoint(27, -1, 10000, {.maxSpeed = 127, .minSpeed = 30});
    intake.move(0);
    push.move(0);
    colors.retract();
    blockDetection.start(&push, &intake, &dist_push_start, &dist_end);
    chassis.moveToPose(44.0, -7.8, -243.3, 900, {.maxSpeed = 100, .minSpeed = 10});
    intake.move(60);
    chassis.swingToHeading(-5, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 80}, false);
    intake.move(-120);

}
/**
 * Runs in driver control
 */
void opcontrol() {
    
    bool flag_loader = true;
    bool flag_descore = true;
    left_motors(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors(pros::E_MOTOR_BRAKE_BRAKE);
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            colors.retract();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            if(dist_end.get_distance() > 0 && dist_end.get_distance() <= 50){
                push.move(0);
                intake.move(127);
            }
            else if(dist_push_start.get_distance() <= 60 && dist_push_start.get_distance() >0 && dist_end.get_distance() >= 50){
                push.tare_position();
                push.move_absolute(95, 100);
                intake.move(127);
                pros::delay(100);
            }
            else {
                intake.move(127);
            }
        }
        if(controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.brake();
            push.brake();
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            colors.retract();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move(-76);
        }
        if(controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.brake();
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            colors.retract();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            push.move(127);
            intake.move(127);
        }
        if(controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L1)){
            push.brake();
            intake.brake();
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            colors.extend();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
                push.move(-46);
                intake.move(-84);
            }
            else{
                intake.move(84);
            }
        }
        if(controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L2)){
            push.brake();
            intake.brake();
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            if(flag_loader){
                loading.extend();
                flag_loader = false;
            }
            else{
                loading.retract();
                flag_loader = true;
            }
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            if(flag_descore){
                dis.extend();
                flag_descore = false;
            }
            else{
                dis.retract();
                flag_descore = true;
            }

        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            forward(chassis, 10);
        }
                if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
                leftMotors.move(90);
                rightMotors.move(90);
                pros::delay(600);}
        // delay to save resources
        pros::delay(10);
    }
}