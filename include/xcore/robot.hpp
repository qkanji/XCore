#pragma once

#include "xcore/command.hpp"
#include "xcore/xdrive.hpp"
#include <cstdint>
#include <vector>

namespace xcore {

/**
 * Base class for a robot's high-level API. Robot-specific repos should derive from this
 * (or compose it) to add their own subsystems and combo functions.
 *
 * Provides the do_()/along() chaining orchestrator described in the fluent API design:
 * `robot.do_(robot.driveTo(...), {along(0.5, [] { ... })})` blocks the calling task until
 * the primary command settles (or times out), firing each trigger once its progress
 * threshold is crossed.
 *
 * NOTE: the design notes this is based on used `robot.do(...)` as the method name, but
 * `do` is a reserved keyword in C++ and cannot be used as an identifier - this is named
 * `do_` instead.
 */
class RobotInterface {
public:
    explicit RobotInterface(XDrive* drive);
    virtual ~RobotInterface() = default;

    /**
     * Returns a heap-allocated drive-to-pose Command bound to this robot's XDrive. Pass it
     * straight into do_(), which takes ownership and frees it once the command settles.
     */
    DriveCommand* driveTo(double x, double y, double theta, uint32_t timeout = 0, double minSpeed = 0,
                           double maxSpeed = 127);

    /**
     * Blocking orchestrator: starts `primary`, polls every 10ms, fires `triggers` at their
     * progress thresholds, and returns (freeing `primary`) once it settles or times out.
     */
    void do_(Command* primary, std::vector<Trigger> triggers = {});

    XDrive& drive() { return *drivePtr; }

private:
    XDrive* drivePtr;
};

} // namespace xcore
