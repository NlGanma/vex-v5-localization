// dismove.cpp — robust version
#include "dismove.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>

// Globals from your project
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Imu imu;
extern pros::Rotation verticalEnc;

// Brake helpers from main.cpp
extern void left_motors(pros::motor_brake_mode_e_t mode);
extern void right_motors(pros::motor_brake_mode_e_t mode);

namespace {
inline double clamp(double v, double lo, double hi) {
    return std::fmax(lo, std::fmin(hi, v));
}
inline int pctTo127(double pct) {
    return static_cast<int>(clamp(pct, -100.0, 100.0) * 1.27);
}
// wrap IMU 0..360 to [-180,180)
inline double wrap180(double h) {
    double a = std::fmod(h + 180.0, 360.0);
    if (a < 0) a += 360.0;
    return a - 180.0;
}
// smallest signed angle difference a-b in degrees, result in [-180,180]
inline double angDiff(double a, double b) {
    double d = std::fmod(a - b, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}
} // namespace

/**
 * Move using a vertical rotation sensor distance target while holding the starting heading.
 * disDeg: target change **in rotation-sensor degrees** (+ forward, - backward)
 * maxPct/minPct: drive percentage caps [0..100]
 * kp: heading correction gain (percent per degree), typical 0.6..1.2
 *
 * Watchdogs:
 *  - Absolute timeout scales with |disDeg|
 *  - Stall timeout exits if no progress for STALL_MS
 */
void dismove_dis_maxspeed_max_min_min_kp_kp(double disDeg,
                                            double maxPct,
                                            double minPct,
                                            double kp) {
    if (disDeg == 0.0) return;

    // sanitize speed limits
    maxPct = std::fabs(maxPct);
    minPct = std::fabs(minPct);
    if (maxPct < 1.0) maxPct = 1.0;
    if (minPct > maxPct) minPct = maxPct;

    // Let chassis coast while we command manually
    left_motors(pros::E_MOTOR_BRAKE_COAST);
    right_motors(pros::E_MOTOR_BRAKE_COAST);

    // Capture references WITHOUT resetting sensors
    const double startPos = verticalEnc.get_position();           // degrees
    const double targetMag = std::fabs(disDeg);                   // magnitude to travel
    const double dir = (disDeg >= 0.0) ? 1.0 : -1.0;              // desired direction
    const double holdHeading = wrap180(imu.get_heading());        // degrees in [-180,180)

    // Timeouts
    const uint32_t t0 = pros::millis();
    // Base 300ms + 0.12ms per degree of rotation sensor travel; clamp to sane limits
    const uint32_t MAX_MS = (uint32_t)clamp(300.0 + 0.12 * targetMag, 400.0, 6000.0);
    const uint32_t STALL_MS = 250;                                // no-progress window

    double covered = 0.0;                                         // progress toward target (>=0)
    double lastCovered = 0.0;
    uint32_t lastProgressT = t0;

    while (covered < targetMag) {
        // Project encoder progress along commanded direction and ignore opposite motion
        double rel = (verticalEnc.get_position() - startPos) * dir;
        covered = rel > 0.0 ? rel : 0.0;

        // Abort on absolute timeout
        uint32_t now = pros::millis();
        if (now - t0 > MAX_MS) break;

        // Stall detection
        if (std::fabs(covered - lastCovered) > 0.5) { // >0.5 deg change counts as progress
            lastCovered = covered;
            lastProgressT = now;
        }
        if (now - lastProgressT > STALL_MS) break;

        // Taper speed from max to min as we approach the target
        double progress = clamp(covered / targetMag, 0.0, 1.0);
        double execMag = maxPct - (maxPct - minPct) * progress;   // linear ramp-down
        double exec = dir * execMag;

        // Heading hold using IMU with wrap-aware error
        const double curHeading = wrap180(imu.get_heading());
        const double headingErr = angDiff(holdHeading, curHeading);   // deg
        double turn = kp * headingErr;                                 // pct
        turn = clamp(turn, -30.0, 30.0);                               // safety cap

        const double leftPct  = clamp(exec + turn, -100.0, 100.0);
        const double rightPct = clamp(exec - turn, -100.0, 100.0);

        leftMotors.move(pctTo127(leftPct));
        rightMotors.move(pctTo127(rightPct));

        pros::delay(5);
    }

    // Stop cleanly before handing control back to LemLib
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(10);
    left_motors(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors(pros::E_MOTOR_BRAKE_BRAKE);
}
