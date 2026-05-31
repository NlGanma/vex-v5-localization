#include <algorithm>
#include <math.h>
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/localization/localization.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

lemlib::OdomSensors::OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
                                 TrackingWheel* horizontal2, pros::Imu* imu)
    : vertical1(vertical1),
      vertical2(vertical2),
      horizontal1(horizontal1),
      horizontal2(horizontal2),
      imu(imu) {}

lemlib::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth,
                               float wheelDiameter, float rpm, float horizontalDrift)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      horizontalDrift(horizontalDrift) {}

lemlib::Chassis::Chassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                         OdomSensors sensors, DriveCurve* throttleCurve, DriveCurve* steerCurve)
    : lateralPID(linearSettings.kP, linearSettings.kI, linearSettings.kD, linearSettings.windupRange, true),
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, true),
      lateralSettings(linearSettings),
      angularSettings(angularSettings),
      drivetrain(drivetrain),
      sensors(sensors),
      throttleCurve(throttleCurve),
      steerCurve(steerCurve),
      lateralLargeExit(lateralSettings.largeError, lateralSettings.largeErrorTimeout),
      lateralSmallExit(lateralSettings.smallError, lateralSettings.smallErrorTimeout),
      angularLargeExit(angularSettings.largeError, angularSettings.largeErrorTimeout),
      angularSmallExit(angularSettings.smallError, angularSettings.smallErrorTimeout) {}

void lemlib::Chassis::configurePto(PtoSettings settings) {
    pto = settings;
    ptoEngaged = false;
    for (int i = 0; i < 4; ++i) ptoRoles[i] = PtoRole::DISABLED;
}

void lemlib::Chassis::engagePto(bool async) {
    if (async) {
        pros::Task task([this]() { engagePto(false); });
        pros::delay(10);
        return;
    }

    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors != nullptr) pto.motorGroups[i].motors->move(0);
    }
    if (pto.piston != nullptr) pto.piston->set_value(pto.engagedValue);

    ptoEngaged = true;
    for (int i = 0; i < 4; ++i) {
        ptoRoles[i] = PtoRole::DISABLED;
        if (pto.motorGroups[i].motors == nullptr) continue;
        pto.motorGroups[i].motors->set_brake_mode_all(getDriveSideBrakeMode(pto.motorGroups[i].driveSide));
    }
}

void lemlib::Chassis::disengagePto(PtoReleaseParams params, bool async) {
    if (async) {
        pros::Task task([this, params]() { disengagePto(params, false); });
        pros::delay(10);
        return;
    }

    if (pto.piston != nullptr) pto.piston->set_value(!pto.engagedValue);

    ptoEngaged = false;
    for (int i = 0; i < 4; ++i) ptoRoles[i] = params.motorRoles[i];

    if (!params.stopMotors) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors != nullptr) pto.motorGroups[i].motors->move(0);
    }
}

void lemlib::Chassis::movePtoRole(PtoRole role, int power) {
    if (ptoEngaged || role == PtoRole::DISABLED) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors != nullptr && ptoRoles[i] == role) pto.motorGroups[i].motors->move(power);
    }
}

void lemlib::Chassis::movePtoGroup(int index, int power) {
    if (ptoEngaged || index < 0 || index >= 4) return;
    if (pto.motorGroups[index].motors != nullptr) pto.motorGroups[index].motors->move(power);
}

bool lemlib::Chassis::isPtoEngaged() const { return ptoEngaged; }

lemlib::PtoRole lemlib::Chassis::getPtoRole(int index) const {
    if (index < 0 || index >= 4) return PtoRole::DISABLED;
    return ptoRoles[index];
}

lemlib::PtoRole lemlib::Chassis::getLeftPtoRole() const {
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors != nullptr && pto.motorGroups[i].driveSide == DriveSide::LEFT) return ptoRoles[i];
    }
    return PtoRole::DISABLED;
}

lemlib::PtoRole lemlib::Chassis::getRightPtoRole() const {
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors != nullptr && pto.motorGroups[i].driveSide == DriveSide::RIGHT)
            return ptoRoles[i];
    }
    return PtoRole::DISABLED;
}

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void calibrateIMU(lemlib::OdomSensors& sensors) {
    constexpr uint32_t kImuAttemptTimeoutMs = 5000;
    int attempt = 1;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5) {
        sensors.imu->reset();
        const uint32_t start = pros::millis();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (sensors.imu->get_status() != pros::ImuStatus::error && sensors.imu->is_calibrating() &&
               pros::millis() - start < kImuAttemptTimeoutMs);
        // exit if imu has been calibrated
        const double heading = sensors.imu->get_heading();
        if (!sensors.imu->is_calibrating() && !std::isnan(heading) && !std::isinf(heading)) break;
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        lemlib::infoSink()->warn("IMU failed to calibrate! Attempt #{}", attempt);
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        sensors.imu = nullptr;
        lemlib::infoSink()->error("IMU calibration failed, defaulting to tracking wheels / motor encoders");
        pros::c::lcd_print(7, "WARNING: IMU FAILED - USING ENCODERS");
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "-.-");
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "IMU calibration failed");
    }
}

void lemlib::Chassis::calibrate(bool calibrateImu) {
    // calibrate the IMU if it exists and the user doesn't specify otherwise
    if (sensors.imu != nullptr && calibrateImu) calibrateIMU(sensors);
    // initialize odom
    if (sensors.vertical1 == nullptr)
        sensors.vertical1 = new lemlib::TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                      -(drivetrain.trackWidth / 2), drivetrain.rpm);
    if (sensors.vertical2 == nullptr)
        sensors.vertical2 = new lemlib::TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                                      drivetrain.trackWidth / 2, drivetrain.rpm);
    sensors.vertical1->reset();
    sensors.vertical2->reset();
    if (sensors.horizontal1 != nullptr) sensors.horizontal1->reset();
    if (sensors.horizontal2 != nullptr) sensors.horizontal2->reset();
    setSensors(sensors, drivetrain);
    init();
    // rumble to controller to indicate success
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void lemlib::Chassis::setControllerSettings(ControllerSettings lateralSettings, ControllerSettings angularSettings) {
    this->lateralSettings = lateralSettings;
    this->angularSettings = angularSettings;

    lateralPID.configure(lateralSettings.kP, lateralSettings.kI, lateralSettings.kD, lateralSettings.windupRange, true);
    angularPID.configure(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, true);

    lateralLargeExit.configure(lateralSettings.largeError, lateralSettings.largeErrorTimeout);
    lateralSmallExit.configure(lateralSettings.smallError, lateralSettings.smallErrorTimeout);
    angularLargeExit.configure(angularSettings.largeError, angularSettings.largeErrorTimeout);
    angularSmallExit.configure(angularSettings.smallError, angularSettings.smallErrorTimeout);
}

void lemlib::Chassis::setPose(float x, float y, float theta, bool radians) {
    lemlib::setPose(lemlib::Pose(x, y, theta), radians);
}

void lemlib::Chassis::setPose(Pose pose, bool radians) { lemlib::setPose(pose, radians); }

lemlib::Pose lemlib::Chassis::getPose(bool radians, bool standardPos) {
    Pose pose = lemlib::getPose(true);
    if (standardPos) pose.theta = M_PI_2 - pose.theta;
    if (!radians) pose.theta = radToDeg(pose.theta);
    return pose;
}

void lemlib::Chassis::waitUntil(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void lemlib::Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void lemlib::Chassis::moveDrive(float left, float right) {
    lastCommandedLeftOutput = left;
    lastCommandedRightOutput = right;
    drivetrain.leftMotors->move(left);
    drivetrain.rightMotors->move(right);
    if (!ptoEngaged) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors == nullptr) continue;
        pto.motorGroups[i].motors->move(pto.motorGroups[i].driveSide == DriveSide::LEFT ? left : right);
    }
}

void lemlib::Chassis::moveDriveSide(DriveSide side, float power) {
    if (side == DriveSide::LEFT) {
        lastCommandedLeftOutput = power;
        drivetrain.leftMotors->move(power);
    } else {
        lastCommandedRightOutput = power;
        drivetrain.rightMotors->move(power);
    }

    if (!ptoEngaged) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors == nullptr || pto.motorGroups[i].driveSide != side) continue;
        pto.motorGroups[i].motors->move(power);
    }
}

void lemlib::Chassis::brakeDriveSide(DriveSide side) {
    if (side == DriveSide::LEFT) {
        lastCommandedLeftOutput = 0.0f;
        drivetrain.leftMotors->brake();
    } else {
        lastCommandedRightOutput = 0.0f;
        drivetrain.rightMotors->brake();
    }

    if (!ptoEngaged) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors == nullptr || pto.motorGroups[i].driveSide != side) continue;
        pto.motorGroups[i].motors->brake();
    }
}

void lemlib::Chassis::stopDrive() { moveDrive(0, 0); }

float lemlib::Chassis::getLastCommandedLeftOutput() const { return lastCommandedLeftOutput.load(); }

float lemlib::Chassis::getLastCommandedRightOutput() const { return lastCommandedRightOutput.load(); }

pros::motor_brake_mode_e lemlib::Chassis::getDriveSideBrakeMode(DriveSide side) const {
    const pros::MotorGroup* motors = side == DriveSide::LEFT ? drivetrain.leftMotors : drivetrain.rightMotors;
    return static_cast<pros::motor_brake_mode_e>(motors->get_brake_mode_all().at(0));
}

void lemlib::Chassis::setDriveSideBrakeMode(DriveSide side, pros::motor_brake_mode_e mode) {
    pros::MotorGroup* motors = side == DriveSide::LEFT ? drivetrain.leftMotors : drivetrain.rightMotors;
    motors->set_brake_mode_all(mode);

    if (!ptoEngaged) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors == nullptr || pto.motorGroups[i].driveSide != side) continue;
        pto.motorGroups[i].motors->set_brake_mode_all(mode);
    }
}

void lemlib::Chassis::requestMotionStart() {
    lemlib::localization::setMotionCorrectionSuppressed(true);
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}

void lemlib::Chassis::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;
    lemlib::localization::setMotionCorrectionSuppressed(this->motionRunning);

    // permit queued motion to run
    this->mutex.give();
}

void lemlib::Chassis::cancelMotion() {
    this->motionRunning = false;
    // cancelMotion only cancels the CURRENT motion, not a queued one. If a
    // motion is still queued it will run next, so keep corrections suppressed;
    // the queued motion's endMotion will recompute suppression from there.
    lemlib::localization::setMotionCorrectionSuppressed(this->motionQueued);
    pros::delay(10); // give time for motion to stop
}

void lemlib::Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    lemlib::localization::setMotionCorrectionSuppressed(false);
    pros::delay(10); // give time for motion to stop
}

bool lemlib::Chassis::isInMotion() const { return this->motionRunning; }

void lemlib::Chassis::resetLocalPosition() {
    float theta = this->getPose().theta;
    lemlib::setPose(lemlib::Pose(0, 0, theta), false);
}

void lemlib::Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    setDriveSideBrakeMode(DriveSide::LEFT, mode);
    setDriveSideBrakeMode(DriveSide::RIGHT, mode);
}

void lemlib::Chassis::drivePulse(float power, int timeout, bool async) {
    const int pulseTime = std::max(timeout, 0);
    if (pulseTime == 0) {
        distTraveled = -1;
        moveDrive(power, power);
        return;
    }

    requestMotionStart();
    if (!this->motionRunning) {
        // Mirror every other motion: release the mutex and restore the
        // suppression flag on the cancelled-before-start path. Without this,
        // a queued drivePulse cancelled before it ran would leak this->mutex
        // (deadlocking all future motions) and leave correction suppression
        // stuck true for the rest of the run.
        this->endMotion();
        return;
    }

    if (async) {
        pros::Task task([=, this]() { drivePulse(power, pulseTime, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    distTraveled = 0;
    moveDrive(power, power);

    lemlib::Timer timer(pulseTime);
    while (!timer.isDone() && motionRunning) pros::delay(10);

    stopDrive();
    distTraveled = -1;
    endMotion();
}
