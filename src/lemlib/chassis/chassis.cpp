#include <algorithm>
#include <math.h>
#include "pros/apix.h"
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
      angularSmallExit(angularSettings.smallError, angularSettings.smallErrorTimeout),
      motionSemaphore(pros::c::sem_create(1, 1)) {}

lemlib::Chassis::~Chassis() {
    if (motionSemaphore != nullptr) pros::c::sem_delete(motionSemaphore);
}

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
    // Close the software gate before changing the piston/roles so no concurrent
    // released-PTO command can race the physical engagement.
    ptoEngaged.store(true);
    if (pto.piston != nullptr) pto.piston->set_value(pto.engagedValue);

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

    for (int i = 0; i < 4; ++i) ptoRoles[i] = params.motorRoles[i];
    if (pto.piston != nullptr) pto.piston->set_value(!pto.engagedValue);
    // Publish the released state only after all roles are ready for readers.
    ptoEngaged.store(false);

    if (!params.stopMotors) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors != nullptr) pto.motorGroups[i].motors->move(0);
    }
}

void lemlib::Chassis::movePtoRole(PtoRole role, int power) {
    if (ptoEngaged.load() || role == PtoRole::DISABLED) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors != nullptr && ptoRoles[i] == role) pto.motorGroups[i].motors->move(power);
    }
}

void lemlib::Chassis::movePtoGroup(int index, int power) {
    if (ptoEngaged.load() || index < 0 || index >= 4) return;
    if (pto.motorGroups[index].motors != nullptr) pto.motorGroups[index].motors->move(power);
}

bool lemlib::Chassis::isPtoEngaged() const { return ptoEngaged.load(); }

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
    while (true) {
        pros::delay(10);
        const float traveled = distTraveled.load();
        if (traveled > dist || traveled == -1.0f) break;
    }
}

void lemlib::Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled.load() != -1.0f);
}

void lemlib::Chassis::moveDrive(float left, float right) {
    lastCommandedLeftOutput = left;
    lastCommandedRightOutput = right;
    drivetrain.leftMotors->move(left);
    drivetrain.rightMotors->move(right);
    if (!ptoEngaged.load()) return;
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

    if (!ptoEngaged.load()) return;
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

    if (!ptoEngaged.load()) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors == nullptr || pto.motorGroups[i].driveSide != side) continue;
        pto.motorGroups[i].motors->brake();
    }
}

void lemlib::Chassis::stopDrive() { moveDrive(0, 0); }

void lemlib::Chassis::noteExternalDriveCommand(float left, float right) {
    lastCommandedLeftOutput = left;
    lastCommandedRightOutput = right;
}

float lemlib::Chassis::getLastCommandedLeftOutput() const { return lastCommandedLeftOutput.load(); }

float lemlib::Chassis::getLastCommandedRightOutput() const { return lastCommandedRightOutput.load(); }

pros::motor_brake_mode_e lemlib::Chassis::getDriveSideBrakeMode(DriveSide side) const {
    const pros::MotorGroup* motors = side == DriveSide::LEFT ? drivetrain.leftMotors : drivetrain.rightMotors;
    return static_cast<pros::motor_brake_mode_e>(motors->get_brake_mode_all().at(0));
}

void lemlib::Chassis::setDriveSideBrakeMode(DriveSide side, pros::motor_brake_mode_e mode) {
    pros::MotorGroup* motors = side == DriveSide::LEFT ? drivetrain.leftMotors : drivetrain.rightMotors;
    motors->set_brake_mode_all(mode);

    if (!ptoEngaged.load()) return;
    for (int i = 0; i < 4; ++i) {
        if (pto.motorGroups[i].motors == nullptr || pto.motorGroups[i].driveSide != side) continue;
        pto.motorGroups[i].motors->set_brake_mode_all(mode);
    }
}

void lemlib::Chassis::requestMotionStart() {
    const uint32_t queueCancelSnapshot = this->motionQueueCancelGeneration.load();
    this->motionQueued.fetch_add(1);
    lemlib::localization::setMotionCorrectionSuppressed(true);
    const bool acquired = motionSemaphore != nullptr && pros::c::sem_wait(motionSemaphore, TIMEOUT_MAX);
    this->motionQueued.fetch_sub(1);
    if (acquired) this->motionOwner.store(pros::c::task_get_current());

    // cancelAllMotions invalidates every request that was already waiting, while
    // cancelMotion invalidates only the active loop. A cancelled waiter still
    // owns the semaphore here so its normal endMotion path can release it.
    const bool cancelledWhileQueued = queueCancelSnapshot != this->motionQueueCancelGeneration.load();
    this->motionRunning.store(acquired && !cancelledWhileQueued);
    this->motionBoundaryReanchorAllowed.store(acquired && !cancelledWhileQueued);
}

void lemlib::Chassis::endMotion() {
    this->motionRunning.store(false);
    const bool boundaryAllowed = this->motionBoundaryReanchorAllowed.exchange(false);
    const bool nextMotionQueued = this->motionQueued.load() > 0;

    if (nextMotionQueued) {
        lemlib::localization::setMotionCorrectionSuppressed(true);
    } else {
        // The drivetrain has already been stopped and this task still owns the
        // motion semaphore, so a staged, fully-gated correction can be committed here
        // without a scheduler-dependent idle delay or any in-motion pose change.
        const bool stagedCommit = boundaryAllowed && lemlib::localization::applyStagedBoundaryReanchor();
        lemlib::localization::setMotionCorrectionSuppressed(false);
        if (boundaryAllowed && !stagedCommit) lemlib::localization::requestBoundaryReanchor();
    }

    // A request can arrive while this function is opening the idle window, so
    // reassert suppression before waking it if a waiter appeared between the
    // two queue reads. A later request sets suppression itself before waiting.
    if (!nextMotionQueued && this->motionQueued.load() > 0) {
        lemlib::localization::setMotionCorrectionSuppressed(true);
    }
    // Permit exactly one queued motion to run only after its pose-injection
    // suppression is guaranteed active.
    this->motionOwner.store(nullptr);
    if (motionSemaphore != nullptr) pros::c::sem_post(motionSemaphore);
}

void lemlib::Chassis::cancelMotion() {
    // bump the generation FIRST: a motion tick-sliced mid-body re-checks
    // motionContinues() and must lose even if a new motion re-raises
    // motionRunning before it gets to run again
    this->motionGeneration.fetch_add(1);
    const bool wasRunning = this->motionRunning.load();
    this->motionBoundaryReanchorAllowed.store(false);
    // cancelMotion only cancels the CURRENT motion, not a queued one. If a
    // motion is still queued it will run next, so keep corrections suppressed;
    // the queued motion's endMotion will recompute suppression from there.
    lemlib::localization::setMotionCorrectionSuppressed(true);
    stopDrive();
    pros::delay(10); // give time for motion to stop
    if (!wasRunning && !this->motionRunning.load() && this->motionQueued.load() == 0) {
        lemlib::localization::setMotionCorrectionSuppressed(false);
        lemlib::localization::clearBoundaryReanchor();
    }
}

void lemlib::Chassis::cancelAllMotions() {
    this->motionGeneration.fetch_add(1);
    this->motionQueueCancelGeneration.fetch_add(1);
    const bool wasRunning = this->motionRunning.load();
    this->motionBoundaryReanchorAllowed.store(false);
    // Keep injection suppressed until the live owner has observed cancellation,
    // stopped the drivetrain, and released the motion semaphore in endMotion().
    lemlib::localization::setMotionCorrectionSuppressed(true);
    stopDrive();
    pros::delay(10); // give time for motion to stop
    if (!wasRunning && !this->motionRunning.load() && this->motionQueued.load() == 0) {
        lemlib::localization::setMotionCorrectionSuppressed(false);
        lemlib::localization::clearBoundaryReanchor();
    }
}

void lemlib::Chassis::recoverInterruptedMotion() {
    if (motionSemaphore == nullptr) return;

    const pros::task_t owner = this->motionOwner.load();
    if (owner != nullptr) {
        const pros::task_state_e_t state = pros::c::task_get_state(owner);
        if (state != pros::E_TASK_STATE_DELETED && state != pros::E_TASK_STATE_INVALID) {
            return;
        }
        this->motionOwner.store(nullptr);
        this->motionRunning.store(false);
        this->motionBoundaryReanchorAllowed.store(false);
        pros::c::sem_post(motionSemaphore);
    } else {
        // A live cancelled motion normally releases within one control tick. Probe
        // for 50 ms; if it remains unavailable and no motion is published, the PROS
        // competition task died while owning it. Unlike a mutex, a semaphore can be
        // released by lifecycle cleanup, and existing waiters remain attached.
        if (pros::c::sem_wait(motionSemaphore, 50)) {
            pros::c::sem_post(motionSemaphore);
        } else if (!this->isInMotion()) {
            // Covers the tiny acquire-to-owner-publication window if its task
            // was killed there.
            pros::c::sem_post(motionSemaphore);
        } else {
            return;
        }
    }

    if (this->motionQueued.load() == 0) {
        lemlib::localization::setMotionCorrectionSuppressed(false);
        lemlib::localization::clearBoundaryReanchor();
    }
}

bool lemlib::Chassis::isInMotion() const { return this->motionRunning.load(); }

bool lemlib::Chassis::motionContinues(uint32_t generation) const {
    return this->motionRunning.load() && generation == this->motionGeneration.load();
}

uint32_t lemlib::Chassis::motionGenerationSnapshot() const { return this->motionGeneration.load(); }

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
        distTraveled.store(-1.0f);
        moveDrive(power, power);
        return;
    }

    requestMotionStart();
    if (!this->motionRunning.load()) {
        // Mirror every other motion: release the semaphore and restore the
        // suppression flag on the cancelled-before-start path. Without this,
        // a queued drivePulse cancelled before it ran would leak motion ownership
        // (deadlocking all future motions) and leave correction suppression
        // stuck true for the rest of the run.
        this->endMotion();
        return;
    }
    const uint32_t motionGen = this->motionGenerationSnapshot();

    if (async) {
        pros::Task task([=, this]() { drivePulse(power, pulseTime, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    distTraveled.store(0.0f);
    if (motionContinues(motionGen)) moveDrive(power, power);

    lemlib::Timer timer(pulseTime);
    while (!timer.isDone() && motionContinues(motionGen)) pros::delay(10);

    stopDrive();
    distTraveled.store(-1.0f);
    endMotion();
}
