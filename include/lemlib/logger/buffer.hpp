#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "pros/rtos.hpp"

namespace lemlib {
/**
 * @brief A buffer implementation
 *
 * Asynchronously processes a backlog of strings at a given rate. The strings are processed in a first in first out
 * order.
 */
class Buffer {
    public:
        /**
         * @brief Construct a new Buffer object
         *
         */
        Buffer(std::function<void(const std::string&)> bufferFunc);

        /**
         * @brief Destroy the Buffer object
         *
         */
        ~Buffer();

        Buffer(const Buffer&) = delete;
        Buffer& operator=(const Buffer&) = delete;

        /**
         * @brief Push to the buffer
         *
         * Holds no lock: the publish runs with the scheduler suspended, so it is atomic against task switches and a
         * producer that PROS deletes mid-push (competition tasks are deleted on every mode change without releasing
         * held mutexes) can only lose its own message, never wedge the buffer.
         *
         * @param bufferData
         */
        void pushToBuffer(std::string bufferData);

        /**
         * @brief Set the rate of the sink
         *
         * @param rate
         */
        void setRate(uint32_t rate);

        /**
         * @brief Check to see if the internal buffer is empty
         *
         */
        bool buffersEmpty();
    private:
        struct Node {
                std::string data;
                Node* next = nullptr;
        };

        /**
         * @brief The function that will be run inside of the buffer's task.
         *
         */
        void taskLoop();

        /**
         * @brief The function that will be applied to each string in the buffer when it is removed.
         *
         */
        std::function<void(std::string)> bufferFunc;

        // producers push newest-first; only the buffer task takes from here
        std::atomic<Node*> inbox {nullptr};
        // oldest-first messages already taken from the inbox; written only by the buffer task
        std::atomic<Node*> pending {nullptr};
        std::atomic<bool> running {true};
        std::atomic<bool> taskStopped {false};

        uint32_t rate = 10;
        // must stay the last member: constructing it starts taskLoop(), which reads every member above
        pros::Task task;
};
} // namespace lemlib
