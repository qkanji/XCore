#pragma once

#include <cstdint>
#include <functional>

namespace xcore {

/**
 * Base type for a single async, cancellable robot action (e.g. a drive movement).
 *
 * NOTE: Command ownership/lifetime is intentionally kept simple (raw pointers, freed by
 * RobotInterface::do_()) for this scaffolding pass. Revisit with smart pointers or a
 * value-type command if this becomes error-prone once real routes are written.
 */
class Command {
public:
    virtual ~Command() = default;

    /** Begins the action asynchronously. Must return immediately. */
    virtual void start() = 0;

    /** Non-blocking: true once the action has settled (including any post-settle delay). */
    virtual bool isSettled() const = 0;

    /** Cancels the action if it's still running. */
    virtual void cancel() = 0;

    /** Progress metric used by along() triggers (e.g. inches traveled for drive commands). */
    virtual double progress() const { return 0; }

    /** Marks this command as a non-terminal waypoint (see DriveCommand::chain()). No-op by default. */
    virtual Command* chain(double minSpeed = 50, double proximityRadius = 3.0) { return this; }

    /** Optional timeout in ms; 0 = no timeout. Enforced by RobotInterface::do_(). */
    uint32_t timeout = 0;
};

/**
 * A progress-triggered side effect used inside RobotInterface::do_(). `progress` is
 * interpreted by the primary Command (e.g. inches traveled from the start position for
 * drive commands).
 */
struct Trigger {
    double progress;
    std::function<void()> action;
};

/** Builds a Trigger. Mirrors the fluent-API example: along(0.5, intake.deploy()). */
Trigger along(double progress, std::function<void()> action);

} // namespace xcore
