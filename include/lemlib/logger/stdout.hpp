#pragma once

#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "lemlib/logger/buffer.hpp"

namespace lemlib {
/**
 * @brief Buffered printing to Stout.
 *
 * LemLib uses a buffered wrapper around stdout in order to guarantee that messages are printed at a constant
 * rate, no matter how many different threads are trying to use the logger. This is a concern  because not every type of
 * connection to the brain has the same amount of bandwidth.
 */
class BufferedStdout : public Buffer {
    public:
        BufferedStdout();

        /**
         * @brief Print a string (thread-safe).
         *
         */
        template <typename... T> void print(fmt::format_string<T...> format, T&&... args) {
            pushToBuffer(fmt::format(format, std::forward<T>(args)...));
        }
};

/**
 * @brief Get the buffered stdout.
 *
 */
BufferedStdout& bufferedStdout();

/**
 * @brief Lock that every task must hold while writing to stdout.
 *
 * The firmware libc's FILE locks are no-op stubs, so two tasks writing stdout at once can corrupt the shared stdout
 * buffer. Never take it from a PROS competition task: PROS deletes those on mode changes without releasing held
 * mutexes.
 */
pros::Mutex& stdoutWriteMutex();
} // namespace lemlib
