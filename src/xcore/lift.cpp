#include "xcore/lift.hpp"
#include <cmath>

namespace xcore {

Lift::Lift(pros::Motor* motor, PID pid, ExitCondition settleExit, pros::motor_brake_mode_e_t brakeMode)
    : motor(motor), pid(pid), settleExit(settleExit) {
    motor->set_brake_mode(brakeMode);
    startTask();
}

void Lift::moveToHeight(double targetIn) {
    target = targetIn;
    pid.reset();
    settleExit.reset();
    holdingTarget = true;
    markBusy();
}

void Lift::moveToHeightSync(double targetIn) {
    moveToHeight(targetIn);
    waitUntilDone();
}

double Lift::getHeight() const { return motor->get_position(); }

bool Lift::step() {
    if (!holdingTarget) return true;

    const float error = static_cast<float>(target) - motor->get_position();
    const float power = pid.update(error);
    motor->move(power);

    const bool reached = settleExit.update(std::fabs(error));
    if (reached) {
        holdingTarget = false;
        motor->move(0);
    }
    return reached;
}

} // namespace xcore
