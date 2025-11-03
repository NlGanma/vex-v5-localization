#pragma once
#include "pros/distance.hpp"
#include "pros/motor_group.hpp" // for pros::MotorGroup
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

class BlockDetection {
  public:
    void start(pros::MotorGroup* push_chain,
               pros::Motor*      intake,
               pros::Distance*   dist_push_start,
               pros::Distance*   dist_end);
    void stop();

    void setPollMs(int ms)     { poll_ms = ms; }
    void setDebounceMs(int ms) { debounce_ms = ms; }

  private:
    pros::Task* task{nullptr};
    bool running{false};
    int poll_ms{10};
    int debounce_ms{100};
};
