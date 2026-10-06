#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

void lemlib::Chassis::turnToHeading(float theta, int timeout, TurnToHeadingParams params, bool async) {
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
        pros::Task task([=, this]() { turnToHeading(theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta;
    float deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float prevTheta = getPose().theta;
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

        // update completion vars: accumulate per-tick heading change, since the
        // net angle from the start wraps at 180 and would count back down on a
        // forced long-way turn
        traveled += fabs(angleError(pose.theta, prevTheta, false));
        prevTheta = pose.theta;
        distTraveled.store(traveled);

        targetTheta = theta;

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
