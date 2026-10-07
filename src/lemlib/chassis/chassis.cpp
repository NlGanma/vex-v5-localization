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

// PROS kernel API (kapi.h, not in the public headers): vTaskSuspendAll/xTaskResumeAll.
extern "C" {
void rtos_suspend_all(void);
int32_t rtos_resume_all(void);
}

namespace {
// Longest single wait of a queued motion for the release wake-up before it
// re-checks whether the motion is free.
constexpr uint32_t kMotionWakePollMs = 10;
} // namespace

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

bool lemlib::Chassis::hasQueuedMotion() const {
    if (this->motionWaiterOverflow.load() > 0) return true;
    for (const auto& waiter : this->motionWaiters) {
        if (waiter.load() != nullptr) return true;
    }
    return false;
}

void lemlib::Chassis::releaseStaleMotionWaiters(pros::task_t task) {
    for (auto& waiter : this->motionWaiters) {
        if (waiter.load() == task) waiter.store(nullptr);
    }
}

void lemlib::Chassis::requestMotionStart() {
    const pros::task_t self = pros::c::task_get_current();
    // An async motion's task already owns the motion, adopted from its
    // caller's request, which also suppressed corrections, published
    // motionRunning and captured the start generation.
    if (this->motionHandoffAdopter.load() == self) {
        this->motionHandoffAdopter.store(nullptr);
        return;
    }
    // Register as a waiter BEFORE suppressing, so no idle decision (endMotion,
    // the cancels, recoverInterruptedMotion) can miss this request and reopen the
    // idle window after its suppression.
    uint32_t queueCancelSnapshot = 0;
    int waiterSlot = -1;
    rtos_suspend_all();
    queueCancelSnapshot = this->motionQueueCancelGeneration.load();
    this->releaseStaleMotionWaiters(self);
    for (int i = 0; i < kMaxMotionWaiters && waiterSlot < 0; ++i) {
        if (this->motionWaiters[i].load() != nullptr) continue;
        this->motionWaiters[i].store(self);
        waiterSlot = i;
    }
    if (waiterSlot < 0) this->motionWaiterOverflow.store(this->motionWaiterOverflow.load() + 1);
    rtos_resume_all();
    lemlib::localization::setMotionCorrectionSuppressed(true);

    // Take the motion, leave the queue and publish the start in one step, so a
    // kill never leaves the motion taken but unowned. cancelAllMotions bumps
    // both generations in one step too: a cancelAllMotions issued after this
    // request began fails either the queue check below or the motion's
    // motionContinues() check. A cancelled waiter still takes the motion here
    // so its normal endMotion path releases it.
    while (true) {
        bool acquired = false;
        rtos_suspend_all();
        if (this->motionOwner.load() == nullptr) {
            acquired = true;
            if (waiterSlot < 0) this->motionWaiterOverflow.store(this->motionWaiterOverflow.load() - 1);
            else if (this->motionWaiters[waiterSlot].load() == self) this->motionWaiters[waiterSlot].store(nullptr);
            this->motionOwner.store(self);
            this->motionStartGeneration.store(this->motionGeneration.load());
            const bool start = queueCancelSnapshot == this->motionQueueCancelGeneration.load();
            this->motionRunning.store(start);
            this->motionBoundaryReanchorAllowed.store(start);
        }
        rtos_resume_all();
        if (acquired) return;
        // The semaphore only wakes waiters. A wake can be lost (a competition
        // task deleted between taking it and re-checking), so wait boundedly.
        if (motionSemaphore != nullptr) pros::c::sem_wait(motionSemaphore, kMotionWakePollMs);
        else pros::delay(kMotionWakePollMs);
    }
}

void lemlib::Chassis::endMotion() {
    const pros::task_t self = pros::c::task_get_current();
    bool owned = false;
    bool boundaryAllowed = false;
    bool nextMotionQueued = false;
    uint32_t recheck = 0;
    rtos_suspend_all();
    owned = this->motionOwner.load() == self;
    if (owned) {
        this->motionRunning.store(false);
        boundaryAllowed = this->motionBoundaryReanchorAllowed.load();
        this->motionBoundaryReanchorAllowed.store(false);
    }
    nextMotionQueued = this->hasQueuedMotion();
    recheck = this->motionIdleRecheck.load();
    rtos_resume_all();
    // Every acquisition ends here exactly once, on the task that owns it. Only
    // recoverInterruptedMotion() takes a motion from its owner, and only from a
    // deleted task, so there is nothing to release otherwise.
    if (!owned) return;
    this->settleAndReleaseMotion(nextMotionQueued, boundaryAllowed, false, recheck);
}

void lemlib::Chassis::settleAndReleaseMotion(bool queued, bool boundaryAllowed, bool clearBoundary,
                                             uint32_t recheck) {
    while (true) {
        if (queued) {
            lemlib::localization::setMotionCorrectionSuppressed(true);
        } else {
            // The drivetrain has already been stopped and this task still owns the
            // motion, so a staged, fully-gated correction can be committed here
            // without a scheduler-dependent idle delay or any in-motion pose change.
            const bool commitAllowed = boundaryAllowed && !clearBoundary;
            const bool stagedCommit = commitAllowed && lemlib::localization::applyStagedBoundaryReanchor();
            lemlib::localization::setMotionCorrectionSuppressed(false);
            if (clearBoundary) lemlib::localization::clearBoundaryReanchor();
            else if (commitAllowed && !stagedCommit) lemlib::localization::requestBoundaryReanchor();
            // A request can arrive while this function is opening the idle window, so
            // reassert suppression before waking it if a waiter appeared. A later
            // request sets suppression itself before it can take the motion.
            if (this->hasQueuedMotion()) lemlib::localization::setMotionCorrectionSuppressed(true);
            boundaryAllowed = false;
        }
        // Release only if the decision above still holds. A cancel's idle settle
        // (after it suppressed) or a recovery (which may have just dropped a
        // deleted task's waiter entry) that found the motion owned or queued has
        // bumped motionIdleRecheck since it was made: decide again, clearing any
        // boundary request. A waiter this pass kept suppression for can also turn
        // out to be a deleted task's entry that recovery has since dropped.
        bool released = false;
        rtos_suspend_all();
        const bool nowQueued = this->hasQueuedMotion();
        const uint32_t nowRecheck = this->motionIdleRecheck.load();
        if (nowRecheck == recheck && (nowQueued || !queued)) {
            this->motionOwner.store(nullptr);
            if (motionSemaphore != nullptr) pros::c::sem_post(motionSemaphore);
            released = true;
        }
        rtos_resume_all();
        if (released) return;
        if (nowRecheck != recheck) clearBoundary = true;
        recheck = nowRecheck;
        queued = nowQueued;
    }
}

void lemlib::Chassis::settleIdleMotion() {
    // Settle while holding the motion, so no request can start between the idle
    // check and the unsuppress. An owner or queued waiter makes that decision
    // itself; the bump makes an owner that already decided decide again.
    bool claimed = false;
    uint32_t recheck = 0;
    rtos_suspend_all();
    if (this->motionOwner.load() == nullptr && !this->hasQueuedMotion()) {
        this->motionOwner.store(pros::c::task_get_current());
        claimed = true;
    } else {
        this->motionIdleRecheck.store(this->motionIdleRecheck.load() + 1);
    }
    recheck = this->motionIdleRecheck.load();
    rtos_resume_all();
    if (claimed) this->settleAndReleaseMotion(false, false, true, recheck);
}

void lemlib::Chassis::cancelMotion() {
    // bump the generation FIRST: a motion tick-sliced mid-body re-checks
    // motionContinues() and must lose even if a new motion re-raises
    // motionRunning before it gets to run again
    rtos_suspend_all();
    this->motionGeneration.store(this->motionGeneration.load() + 1);
    this->motionBoundaryReanchorAllowed.store(false);
    rtos_resume_all();
    // cancelMotion only cancels the CURRENT motion, not a queued one. If a
    // motion is still queued it will run next, so keep corrections suppressed;
    // the queued motion's endMotion will recompute suppression from there.
    lemlib::localization::setMotionCorrectionSuppressed(true);
    // Sample AFTER suppressing: a running owner then clears motionRunning and
    // restores suppression in endMotion after this write; an owner that already
    // ended is seen as idle and restored below.
    const bool wasRunning = this->motionRunning.load();
    stopDrive();
    pros::delay(10); // give time for motion to stop
    if (!wasRunning) this->settleIdleMotion();
}

void lemlib::Chassis::cancelAllMotions() {
    // Both generations in one step: requestMotionStart takes its queue-cancel
    // snapshot and its start generation in separate steps around this one.
    rtos_suspend_all();
    this->motionQueueCancelGeneration.store(this->motionQueueCancelGeneration.load() + 1);
    this->motionGeneration.store(this->motionGeneration.load() + 1);
    this->motionBoundaryReanchorAllowed.store(false);
    rtos_resume_all();
    // Keep injection suppressed until the live owner has observed cancellation,
    // stopped the drivetrain, and released the motion in endMotion().
    lemlib::localization::setMotionCorrectionSuppressed(true);
    // Sampled after suppressing, as in cancelMotion.
    const bool wasRunning = this->motionRunning.load();
    stopDrive();
    pros::delay(10); // give time for motion to stop
    if (!wasRunning) this->settleIdleMotion();
}

void lemlib::Chassis::recoverInterruptedMotion() {
    // PROS recreates every competition task in one static TCB, so a predecessor
    // it deleted mid-wait or mid-motion has this caller's handle, and
    // task_get_state() would report it RUNNING. Callers run at a competition
    // callback entry before starting any motion, so that handle is the dead task.
    const pros::task_t self = pros::c::task_get_current();
    bool claimed = false;
    bool queued = false;
    uint32_t recheck = 0;
    // One scheduler-suspended step, also against adoptMotionHandoff(): the async
    // task of a killed caller has either taken over (a live owner) or its hand-off
    // is voided here and it never runs. The owner's state is read inside the step
    // too (task_get_state only reads its state list in a critical section), so a
    // finished task's handle reused by a new owner cannot pass for the dead one.
    rtos_suspend_all();
    this->releaseStaleMotionWaiters(self);
    const pros::task_t owner = this->motionOwner.load();
    bool ownerDead = owner == self;
    if (owner != nullptr && owner != self) {
        const pros::task_state_e_t state = pros::c::task_get_state(owner);
        ownerDead = state == pros::E_TASK_STATE_DELETED || state == pros::E_TASK_STATE_INVALID;
    }
    if (ownerDead) {
        this->motionOwner.store(self);
        this->motionHandoffToken.store(0);
        this->motionRunning.store(false);
        this->motionBoundaryReanchorAllowed.store(false);
        this->distTraveled.store(-1.0f);
        claimed = true;
    } else if (owner == nullptr && !this->hasQueuedMotion()) {
        this->motionOwner.store(self);
        claimed = true;
    } else {
        this->motionIdleRecheck.store(this->motionIdleRecheck.load() + 1);
    }
    queued = this->hasQueuedMotion();
    recheck = this->motionIdleRecheck.load();
    rtos_resume_all();
    if (claimed) this->settleAndReleaseMotion(queued, false, true, recheck);
}

bool lemlib::Chassis::isInMotion() const { return this->motionRunning.load(); }

bool lemlib::Chassis::motionContinues(uint32_t generation) const {
    return this->motionRunning.load() && generation == this->motionGeneration.load();
}

uint32_t lemlib::Chassis::motionGenerationSnapshot() const { return this->motionStartGeneration.load(); }

void lemlib::Chassis::startAsyncMotion(std::function<void()> motion) {
    // The caller still owns the motion from its requestMotionStart().
    // Handing it over, instead of releasing it for the new task to queue again,
    // means no other motion can start in between and the task keeps the start
    // generation captured there: any cancelMotion() issued since that request
    // took the motion (any cancelAllMotions() since it registered as a waiter)
    // stops it, and no cancel aimed at another motion can.
    const pros::task_t parent = pros::c::task_get_current();
    uint32_t token = this->motionHandoffSerial.load() + 1;
    if (token == 0) token = 1;
    this->motionHandoffSerial.store(token);
    this->motionHandoffToken.store(token);
    pros::Task task([this, token, motion = std::move(motion)]() {
        if (!this->adoptMotionHandoff(token)) return;
        motion();
        // Every motion consumes the adoption in its requestMotionStart(); one that
        // returned before it would otherwise keep the motion forever.
        if (this->motionHandoffAdopter.load() == pros::c::task_get_current()) {
            this->motionHandoffAdopter.store(nullptr);
            this->endMotion();
        }
    });
    const pros::task_t child = static_cast<pros::task_t>(task);
    if (child == nullptr) {
        // task creation failed: nothing will adopt the motion, so release it here
        this->motionHandoffToken.store(0);
        this->endMotion();
    } else {
        // Publish the new task as owner before returning: if this caller is a
        // competition task, recoverInterruptedMotion() on its successor (same
        // handle) would otherwise reclaim a live task's motion.
        rtos_suspend_all();
        if (this->motionHandoffToken.load() == token && this->motionOwner.load() == parent) {
            this->motionOwner.store(child);
        }
        rtos_resume_all();
    }
    pros::delay(10); // delay to give the task time to start
}

bool lemlib::Chassis::adoptMotionHandoff(uint32_t token) {
    const pros::task_t self = pros::c::task_get_current();
    // The token, not the owner handle, identifies the hand-off: a killed caller's
    // successor reuses its handle. Every reclaim clears the token, and both sides
    // run scheduler-suspended (not ldrex/strex, see logger/buffer.cpp), so this
    // task either takes over before recoverInterruptedMotion() reclaims the
    // motion or never runs.
    rtos_suspend_all();
    const bool adopted = this->motionHandoffToken.load() == token;
    if (adopted) {
        this->motionHandoffToken.store(0);
        this->motionOwner.store(self);
        this->motionHandoffAdopter.store(self);
    }
    rtos_resume_all();
    return adopted;
}

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
        // Mirror every other motion: release the motion and restore the
        // suppression flag on the cancelled-before-start path. Without this,
        // a queued drivePulse cancelled before it ran would leak motion ownership
        // (deadlocking all future motions) and leave correction suppression
        // stuck true for the rest of the run.
        this->endMotion();
        return;
    }
    const uint32_t motionGen = this->motionGenerationSnapshot();

    if (async) {
        startAsyncMotion([=, this]() { drivePulse(power, pulseTime, false); });
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
