#pragma once

#include "pros/rtos.hpp"
#include <atomic>
#include <cstdint>

namespace xcore {

/**
 * Base class for all hardware-owning subsystems (drivetrain, lift, pneumatics, ...).
 *
 * Subclasses own their hardware privately and are only ever driven through their own
 * high-level methods. An internal 10ms task loop repeatedly calls step() to update
 * hardware toward the current target and to maintain the busy flag:
 *  - markBusy() is called by the subclass whenever it accepts a new target.
 *  - step() returns true the instant the target has been reached.
 *  - busy stays true for settleDelayMs() after that, to let the mechanism physically
 *    settle before being reported free again.
 */
class Subsystem {
public:
    Subsystem() = default;
    virtual ~Subsystem();

    /** True from the moment a new target is accepted until reached + settled. */
    bool isBusy() const;

    /** Blocks the calling task until isBusy() is false, or timeoutMs elapses (0 = no timeout). */
    void waitUntilDone(uint32_t timeoutMs = 0) const;

protected:
    /**
     * Called every 10ms from the internal task. Should drive hardware toward the current
     * target and return true once the target has just been reached (busy stays true
     * through the settle delay below).
     */
    virtual bool step() = 0;

    /** How long to keep `busy` true after step() first reports "reached". Default 75ms. */
    virtual uint32_t settleDelayMs() const { return 75; }

    /** Call whenever a new target is accepted, before/when starting motion toward it. */
    void markBusy();

    /** Starts the internal task loop. Call once from the derived class's constructor. */
    void startTask();

private:
    void taskLoop();

    std::atomic<bool> busy{false};
    pros::Task* task = nullptr;
    int64_t reachedAtMs = -1;
};

} // namespace xcore
