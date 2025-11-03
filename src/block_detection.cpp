#include "Customfn/block_detection.hpp"
#include "pros/rtos.hpp"

// File-local args container (not nested, not exposed)
namespace {
struct BlockDetectionArgs {
    pros::MotorGroup* push;
    pros::Motor*      intake;
    pros::Distance*   s_push;   // dist_push_start
    pros::Distance*   s_end;    // dist_end
    bool*             running;
    int               poll_ms;
    int               debounce_ms;
};
inline bool valid_mm(int mm) { return mm > 0; }

void blockDetectionTaskFn(void* vp) {
    auto* a = static_cast<BlockDetectionArgs*>(vp);

    while (*a->running) {
        const int end_mm  = a->s_end->get_distance();
        const int push_mm = a->s_push->get_distance();

        if (valid_mm(end_mm) && end_mm <= 50) {
            a->push->move(0);
            a->intake->move(100);
        } else if (valid_mm(push_mm) && push_mm <= 60 && end_mm >= 50) {
            a->push->tare_position();
            a->push->move_absolute(95, 100);
            a->intake->move(127);
            pros::delay(a->debounce_ms);
        } else {
            a->intake->move(127);
        }

        pros::delay(a->poll_ms); // yield this task only
    }

    delete a; // free heap args
}
} // namespace

void BlockDetection::start(pros::MotorGroup* push_chain,
                           pros::Motor*      intake,
                           pros::Distance*   dist_push_start,
                           pros::Distance*   dist_end) {
    stop(); // if already running

    running = true;
    auto* a = new BlockDetectionArgs{
        .push        = push_chain,
        .intake      = intake,
        .s_push      = dist_push_start,
        .s_end       = dist_end,
        .running     = &running,
        .poll_ms     = poll_ms,
        .debounce_ms = debounce_ms,
    };
    task = new pros::Task(blockDetectionTaskFn, static_cast<void*>(a), "block-detect");
}

void BlockDetection::stop() {
    if (!task) return;
    running = false;
    pros::delay(20);
    delete task;
    task = nullptr;
}
