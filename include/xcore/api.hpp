#pragma once

// XCore: fluent command-chaining API, X-drive motion algorithms, and generic subsystem
// classes for VEX V5 (PROS). Include this single header to get everything.
//
// XCore depends on LemLib for its PID, Pose, and ExitCondition primitives - apply LemLib to
// your project before applying XCore:
//   pros c add-depot LemLib https://raw.githubusercontent.com/LemLib/LemLib/depot/stable.json
//   pros c apply LemLib

#include "xcore/command.hpp"
#include "xcore/config.hpp"
#include "xcore/lift.hpp"
#include "xcore/mcl.hpp"
#include "xcore/pneumatics.hpp"
#include "xcore/robot.hpp"
#include "xcore/subsystem.hpp"
#include "xcore/types.hpp"
#include "xcore/xdrive.hpp"
