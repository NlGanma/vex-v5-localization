// include/lemlib/chassis/forward.hpp
#pragma once
#include "lemlib/api.hpp"
#include <cmath>

inline void forward(lemlib::Chassis& chassis, double inches,
                    int timeout_ms = 2000,
                    lemlib::MoveToPointParams params = {}) {
    const auto s = chassis.getPose();                 // inches, degrees
    const double r = s.theta * M_PI / 180.0;
    const double xT = s.x + inches * std::cos(r);
    const double yT = s.y + inches * std::sin(r);

    // drive forward for +dist, drive backward for -dist (no spin-around)
    params.forwards = inches >= 0;
    chassis.moveToPoint(xT, yT, timeout_ms, params);
    chassis.waitUntilDone();
}
