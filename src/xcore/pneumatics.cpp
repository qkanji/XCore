#include "xcore/pneumatics.hpp"
#include "pros/rtos.hpp"

namespace xcore {

Pneumatics::Pneumatics(pros::adi::Pneumatics* piston, uint32_t actuationTimeMs)
    : piston(piston), actuationTimeMs(actuationTimeMs) {
    startTask();
}

void Pneumatics::extend() { set(true); }

void Pneumatics::retract() { set(false); }

void Pneumatics::set(bool extended) {
    if (piston->is_extended() == extended) return;
    if (extended) {
        piston->extend();
    } else {
        piston->retract();
    }
    actuatedAt = pros::millis();
    moving = true;
    markBusy();
}

bool Pneumatics::isExtended() const { return piston->is_extended(); }

bool Pneumatics::step() {
    if (!moving) return true;
    const bool reached = pros::millis() - actuatedAt >= actuationTimeMs;
    if (reached) moving = false;
    return reached;
}

} // namespace xcore
