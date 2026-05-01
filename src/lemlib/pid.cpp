#include "lemlib/pid.hpp"
#include "lemlib/util.hpp"

namespace lemlib {
PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

void PID::configure(float kP, float kI, float kD, float windupRange, bool signFlipReset) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->windupRange = windupRange;
    this->signFlipReset = signFlipReset;
    reset();
}

float PID::update(const float error) {
    // calculate integral
    integral += error;
    if (sgn(error) != sgn((prevError)) && signFlipReset) integral = 0;
    if (fabs(error) > windupRange && windupRange != 0) integral = 0;

    // calculate derivative
    const float derivative = error - prevError;
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}
} // namespace lemlib
