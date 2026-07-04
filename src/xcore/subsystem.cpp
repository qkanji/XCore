#include "xcore/subsystem.hpp"

namespace xcore {

Subsystem::~Subsystem() {
    if (task != nullptr) {
        task->remove();
        delete task;
    }
}

bool Subsystem::isBusy() const { return busy.load(); }

void Subsystem::waitUntilDone(uint32_t timeoutMs) const {
    const uint32_t start = pros::millis();
    while (isBusy()) {
        if (timeoutMs > 0 && pros::millis() - start >= timeoutMs) break;
        pros::delay(5);
    }
}

void Subsystem::markBusy() {
    busy = true;
    reachedAtMs = -1;
}

void Subsystem::startTask() {
    task = new pros::Task([this] { taskLoop(); });
}

void Subsystem::taskLoop() {
    while (true) {
        const bool reached = step();
        if (reached) {
            const int64_t now = pros::millis();
            if (reachedAtMs < 0) reachedAtMs = now;
            if (now - reachedAtMs >= static_cast<int64_t>(settleDelayMs())) { busy = false; }
        } else {
            reachedAtMs = -1;
            busy = true;
        }
        pros::delay(10);
    }
}

} // namespace xcore
