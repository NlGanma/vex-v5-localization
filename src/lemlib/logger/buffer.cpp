#define FMT_HEADER_ONLY
#include "fmt/core.h"
#include "lemlib/logger/buffer.hpp"
#include "lemlib/logger/message.hpp"

namespace lemlib {
Buffer::Buffer(std::function<void(const std::string&)> bufferFunc)
    : bufferFunc(bufferFunc),
      task([this]() { taskLoop(); }) {}

bool Buffer::buffersEmpty() {
    mutex.take();
    bool status = buffer.size() == 0;
    mutex.give();
    return status;
}

Buffer::~Buffer() {
    running = false;
    while (!taskStopped.load()) { pros::delay(10); }
}

void Buffer::pushToBuffer(const std::string& bufferData) {
    mutex.take();
    buffer.push_back(bufferData);
    mutex.give();
}

void Buffer::setRate(uint32_t rate) { this->rate = rate; }

void Buffer::taskLoop() {
    while (running.load() || !buffersEmpty()) {
        std::string nextMessage;
        bool hasMessage = false;
        mutex.take();
        if (buffer.size() > 0) {
            nextMessage = std::move(buffer.front());
            buffer.pop_front();
            hasMessage = true;
        }
        mutex.give();
        if (hasMessage) bufferFunc(nextMessage);
        pros::delay(rate);
    }
    taskStopped = true;
}
} // namespace lemlib
