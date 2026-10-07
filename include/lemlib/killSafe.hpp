#pragma once

#include <atomic>
#include <cstdint>
#include <utility>

#include "pros/rtos.hpp"

// PROS kernel API (kapi.h, not in the public headers): vTaskSuspendAll/xTaskResumeAll.
extern "C" {
void rtos_suspend_all(void);
int32_t rtos_resume_all(void);
}

namespace lemlib {
namespace detail {
inline bool tryTakeAll() { return true; }

// Zero-timeout takes in order; on a failure, gives back the ones already taken (legal
// inside the region) and reports false.
template <typename... Rest> bool tryTakeAll(pros::Mutex& first, Rest&... rest) {
    if (!first.take(0)) return false;
    if (tryTakeAll(rest...)) return true;
    first.give();
    return false;
}

inline void giveAll() {}

template <typename... Rest> void giveAll(pros::Mutex& first, Rest&... rest) {
    giveAll(rest...);
    first.give();
}

// Out of line so each body is one whole function that runs entirely inside the
// region, and can be audited as such in the disassembly.
template <typename Body> [[gnu::noinline]] void runKillSafeBody(Body& body) noexcept { body(); }
} // namespace detail

/**
 * @brief Run `body` holding every mutex in `mutexes`, all taken (zero timeout) inside
 * one scheduler-suspended region.
 *
 * PROS deletes a competition task on a mode change without releasing the mutexes it
 * owns. While the scheduler is suspended no task runs, the deleting daemon included,
 * so the deletion lands before the region (nothing taken) or after it (everything
 * given). Contention is waited out outside the region, holding nothing. Competition-
 * task paths take shared mutexes only through this; persistent-task-only blocking
 * sections are marked at their mutex declarations.
 *
 * `body` runs with the scheduler suspended, so it must be short (about 50 us at most)
 * and noexcept. Allowed: plain and std::atomic loads/stores/read-modify-writes, copies
 * and value-initialisation of trivially copyable structs, pros::millis(), pure math,
 * TrackingWheel::getType/getOffset, std::deque::swap/pop_front, operator delete/free,
 * std::string::clear(), and an abortIf pointer whose body is one atomic load.
 * Forbidden (each can configASSERT, block or spin with the scheduler suspended, or
 * overrun the budget): any delay, a take/wait/queue call with a nonzero timeout,
 * printf and logger calls, device reads and writes, pros::screen and controller
 * calls, task create/delete/notify, file I/O. Also forbidden: any allocation and
 * anything that can throw. A successful malloc/kmalloc would nest harmlessly
 * (__malloc_lock and kmalloc nest rtos_suspend_all), but a failed operator new
 * throws into this noexcept body (std::terminate), a failed kmalloc masks interrupts
 * and spins in vApplicationMallocFailedHook, and a heap walk can overrun the budget.
 *
 * A pros::Mutex is created lazily on its first-ever take, so the take(0) below may
 * create it inside the region (lazy_init -> mutex_create -> kmalloc). That is safe:
 * PROS already does it in a critical section, kmalloc nests the suspension, and the
 * creation sends with zero ticks. No mutex needs pre-touching.
 *
 * Never call a user of this from inside any suspended region (another withKillSafeLock
 * body, or the rtos_suspend_all() steps in chassis.cpp, logger/buffer.cpp and the
 * localization task's trace-flag read-and-clear in localization/localization.cpp), nor
 * while the calling task holds one of the same mutexes through a blocking take:
 * take(0) on it would never succeed. The scheduler must be running.
 */
template <typename Body, typename... Mutexes>
[[gnu::noinline]] void withKillSafeLock(Body&& body, Mutexes&... mutexes) {
    static_assert(noexcept(body()), "withKillSafeLock body must be noexcept");
    for (uint32_t attempt = 0;; ++attempt) {
        rtos_suspend_all();
        std::atomic_signal_fence(std::memory_order_seq_cst);
        if (detail::tryTakeAll(mutexes...)) {
            detail::runKillSafeBody(body);
            std::atomic_signal_fence(std::memory_order_seq_cst);
            detail::giveAll(mutexes...);
            rtos_resume_all();
            return;
        }
        rtos_resume_all();
        // An equal-priority holder that was preempted mid-section usually finishes
        // within a yield; a blocked one (device read, nested lock) gets a tick.
        pros::delay(attempt < 2 ? 0 : 1);
    }
}
} // namespace lemlib
