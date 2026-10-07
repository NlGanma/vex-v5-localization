#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

void lemlib::Chassis::turnToPoint(float x, float y, int timeout, TurnToPointParams params, bool async) {
    params.minSpeed = std::abs(params.minSpeed);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning.load()) {
        this->endMotion();
        return;
    }
    // capture the motion generation: motionContinues() makes a cancelled loop
    // exit even if a new motion re-raises motionRunning before this one re-checks
    const uint32_t motionGen = this->motionGenerationSnapshot();
    // if the function is async, run it in a new task
    if (async) {
        this->startAsyncMotion([=, this]() { turnToPoint(x, y, timeout, params, false); });
        return;
    }
    float targetTheta;
    float deltaX, deltaY, deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float prevTheta = getPose().theta;
    // distTraveled accumulates heading change against the loop pose, which gets
    // a -180 facing adjustment for backwards turns; the seed needs the same
    // shift or the first tick adds ~180 and fires waitUntil at once.
    if (!params.forwards) prevTheta = fmod(prevTheta - 180, 360);
    float traveled = 0;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    distTraveled.store(0.0f);
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionContinues(motionGen)) {
        // update variables
        Pose pose = getPose();
        pose.theta = (params.forwards) ? fmod(pose.theta, 360) : fmod(pose.theta - 180, 360);

        // update completion vars: accumulate per-tick heading change, since the
        // net angle from the start wraps at 180 and would count back down on a
        // forced long-way turn
        traveled += fabs(angleError(pose.theta, prevTheta, false));
        prevTheta = pose.theta;
        distTraveled.store(traveled);

        deltaX = x - pose.x;
        deltaY = y - pose.y;
        targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        // motion chaining
        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        infoSink()->debug("Turn Motor Power: {} ", motorPower);

        // move the drivetrain
        if (!motionContinues(motionGen)) break;
        moveDrive(motorPower, -motorPower);

        pros::delay(10);
    }

    // stop the drivetrain
    stopDrive();
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled.store(-1.0f);
    this->endMotion();
}
