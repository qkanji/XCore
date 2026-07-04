#pragma once

#include "lemlib/exitcondition.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/pose.hpp"

namespace xcore {

// XCore builds on LemLib's generic PID, Pose, and ExitCondition (settle-detection)
// primitives instead of re-implementing them - only genuinely X-drive-specific logic
// (motor mixing, holonomic odometry) is original to XCore. LemLib must be applied to any
// project that uses XCore:
//   pros c add-depot LemLib https://raw.githubusercontent.com/LemLib/LemLib/depot/stable.json
//   pros c apply LemLib
// before applying XCore.
using lemlib::ExitCondition;
using lemlib::PID;
using lemlib::Pose;

} // namespace xcore
