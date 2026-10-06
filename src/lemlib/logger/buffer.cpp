#define FMT_HEADER_ONLY
#include "fmt/core.h"
#include "lemlib/logger/buffer.hpp"
#include "lemlib/logger/message.hpp"

// PROS kernel API (kapi.h, not in the public headers): vTaskSuspendAll/xTaskResumeAll.
extern "C" {
void rtos_suspend_all(void);
int32_t rtos_resume_all(void);
}

namespace lemlib {
Buffer::Buffer(std::function<void(const std::string&)> bufferFunc)
    : bufferFunc(bufferFunc),
      task([this]() { taskLoop(); }) {}

bool Buffer::buffersEmpty() { return inbox.load() == nullptr && pending.load() == nullptr; }

Buffer::~Buffer() {
    running = false;
    while (!taskStopped.load()) { pros::delay(10); }
}

void Buffer::pushToBuffer(std::string bufferData) {
    // allocate outside the region: nothing in a scheduler-suspended region may block.
    // Suspending the scheduler, not ldrex/strex, makes the publish atomic: the PROS
    // context switch never clears the exclusive monitor, so a preempted strex could
    // succeed on another task's reservation and drop a message.
    Node* node = new Node {std::move(bufferData), nullptr};
    rtos_suspend_all();
    node->next = inbox.load(std::memory_order_relaxed);
    inbox.store(node, std::memory_order_release);
    rtos_resume_all();
}

void Buffer::setRate(uint32_t rate) { this->rate = rate; }

void Buffer::taskLoop() {
    while (running.load() || !buffersEmpty()) {
        Node* next = pending.load(std::memory_order_relaxed);
        if (next == nullptr) {
            rtos_suspend_all();
            Node* batch = inbox.load(std::memory_order_acquire);
            inbox.store(nullptr, std::memory_order_relaxed);
            rtos_resume_all();
            // reverse the newest-first inbox into first in first out order
            while (batch != nullptr) {
                Node* older = batch->next;
                batch->next = next;
                next = batch;
                batch = older;
            }
        }
        if (next != nullptr) {
            pending.store(next->next, std::memory_order_relaxed);
            bufferFunc(std::move(next->data));
            delete next;
        }
        pros::delay(rate);
    }
    taskStopped = true;
}
} // namespace lemlib
