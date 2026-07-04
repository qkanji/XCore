#include "xcore/config.hpp"

namespace xcore::config {

// TODO: placeholder gains - tune on the real robot.
const PID LATERAL_PID(10, 0, 1);
const ExitCondition LATERAL_SMALL_EXIT(1, 100);
const ExitCondition LATERAL_LARGE_EXIT(3, 500);

const PID ANGULAR_PID(3, 0, 0.3);
const ExitCondition ANGULAR_SMALL_EXIT(1, 100);
const ExitCondition ANGULAR_LARGE_EXIT(3, 500);

const PID LIFT_PID(1, 0, 0.1);
const ExitCondition LIFT_EXIT(2, 100);

const MCLConfig MCL_SETTINGS {}; // defaults for now - tune particle count/noise on the real robot

} // namespace xcore::config
