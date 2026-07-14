#include <cmath>
#include <optional>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

void lemlib::Chassis::moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params, bool async) {
    // take the mutex
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
        pros::Task task([=, this]() { moveToPose(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    // calculate target pose in standard form
    Pose target(x, y, M_PI_2 - degToRad(theta));
    if (!params.forwards) target.theta = fmod(target.theta + M_PI, 2 * M_PI); // backwards movement

    // use global horizontalDrift is horizontalDrift is 0
    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    // initialize vars used between iterations
    Pose lastPose = getPose();
    distTraveled.store(0.0f);
    Timer timer(timeout);
    bool close = false;
    bool lateralSettled = false;
    std::optional<bool> prevRobotSide = std::nullopt;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power

    // main loop
    while (!timer.isDone() &&
           ((!lateralSettled || (!angularLargeExit.getExit() && !angularSmallExit.getExit())) || !close) &&
           this->motionContinues(motionGen)) {
        // update position
        const Pose pose = getPose(true, true);

        // update distance traveled
        distTraveled.fetch_add(pose.distance(lastPose));
        lastPose = pose;

        // calculate distance to the target point
        const float distTarget = pose.distance(target);

        // check if the robot is close enough to the target to start settling
        if (distTarget < 7.5 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }

        // check if the lateral controller has settled
        if (lateralLargeExit.getExit() && lateralSmallExit.getExit()) lateralSettled = true;

        // calculate the carrot point
        Pose carrot = target - Pose(cos(target.theta), sin(target.theta)) * params.lead * distTarget;
        if (close) carrot = target; // settling behavior

        // motion chaining: exit when the robot itself crosses the half-plane
        // earlyExitRange before the target, mirroring moveToPoint's crossing
        // check. The old robot-vs-carrot sameSide comparison broke at the close
        // latch: teleporting the carrot to the target flipped carrotSide there,
        // firing the exit at the fixed 7.5" latch instead of at earlyExitRange.
        const bool robotSide =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevRobotSide == std::nullopt) prevRobotSide = robotSide;
        // Exit on the configured crossing. This may intentionally occur before
        // the fixed 7.5-inch settling latch when earlyExitRange is larger.
        if (robotSide != prevRobotSide && params.minSpeed != 0) break;
        prevRobotSide = robotSide;

        // calculate error
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError =
            close ? angleError(adjustedRobotTheta, target.theta) : angleError(adjustedRobotTheta, pose.angle(carrot));
        float lateralError = pose.distance(carrot);
        // only use cos when settling
        // otherwise just multiply by the sign of cos
        // maxSlipSpeed takes care of lateralOut
        if (close) lateralError *= cos(angleError(pose.theta, pose.angle(carrot)));
        else lateralError *= sgn(cos(angleError(pose.theta, pose.angle(carrot))));

        // update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);
        angularSmallExit.update(radToDeg(angularError));
        angularLargeExit.update(radToDeg(angularError));

        // get output from PIDs
        float lateralOut = lateralPID.update(lateralError);
        float angularOut = angularPID.update(radToDeg(angularError));

        // apply restrictions on angular speed
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);

        // rate-limit angular output in the large-error regime, mirroring
        // turnToHeading's slew. The angular reference here is the bearing to the
        // moving carrot point, so the unslewed kD term can inject a large
        // per-loop kick that pushes the heading into a divergent oscillation.
        // Guarded by >20 deg (same threshold turnToHeading uses) so fine
        // small-angle settling keeps full PID authority.
        // Re-clamp after the slew: slew() also rate-limits DECREASES, so when the
        // close latch shrinks params.maxSpeed the slewed value can sit above the
        // new cap for a few ticks, and the overturn subtraction below would then
        // exceed |lateralOut| and briefly reverse the drive.
        if (fabs(radToDeg(angularError)) > 20)
            angularOut =
                std::clamp(slew(angularOut, prevAngularOut, angularSettings.slew), -params.maxSpeed, params.maxSpeed);
        prevAngularOut = angularOut;

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        // constrain lateral output by max accel
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // constrain lateral output by the max speed it can travel at without
        // slipping
        const float radius = 1 / fabs(getCurvature(pose, carrot));
        const float maxSlipSpeed(sqrt(params.horizontalDrift * radius * 9.8));
        lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
        // prioritize angular movement over lateral movement
        const float overturn = fabs(angularOut) + fabs(lateralOut) - params.maxSpeed;
        if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

        // prevent moving in the wrong direction
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        // constrain lateral output by the minimum speed
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        // update previous output
        prevLateralOut = lateralOut;

        infoSink()->debug("lateralOut: {} angularOut: {}", lateralOut, angularOut);

        // ratio the speeds to respect the max speed
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move the drivetrain
        if (!motionContinues(motionGen)) break;
        moveDrive(leftPower, rightPower);

        // delay to save resources
        pros::delay(10);
    }

    // stop the drivetrain
    stopDrive();
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled.store(-1.0f);
    this->endMotion();
}
