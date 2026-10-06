#include <iostream>
#include <ostream>

#include "lemlib/logger/stdout.hpp"

namespace lemlib {
namespace {
// constinit: initialized at compile time, so no lazy static guard is involved
constinit pros::Mutex stdoutWriteMutexInstance;
} // namespace

pros::Mutex& stdoutWriteMutex() { return stdoutWriteMutexInstance; }

BufferedStdout::BufferedStdout()
    : Buffer([](const std::string& text) {
          stdoutWriteMutex().take();
          std::cout << text << std::flush;
          stdoutWriteMutex().give();
      }) {
    setRate(50);
}

BufferedStdout& bufferedStdout() {
    static BufferedStdout bufferedStdout;
    return bufferedStdout;
}
} // namespace lemlib