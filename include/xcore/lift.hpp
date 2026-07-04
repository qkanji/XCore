#pragma once

#include "pros/motors.hpp"
#include "xcore/subsystem.hpp"
#include "xcore/types.hpp"

namespace xcore {

/**
 * Generic single-motor lift (or arm) subsystem with PID position control, built on
 * lemlib::PID plus a lemlib::ExitCondition for settle detection.
 *
 * Height/angle units are whatever the caller's target values are in (e.g. degrees of
 * motor/rotation-sensor travel) - XCore does not assume a specific sensor.
 */
class Lift : public Subsystem {
public:
    Lift(pros::Motor* motor, PID pid, ExitCondition settleExit,
         pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_HOLD);

    /** Async: begins moving toward the target position. Returns immediately. */
    void moveToHeight(double target);

    /** Sync: moveToHeight(...) followed by waitUntilDone(). */
    void moveToHeightSync(double target);

    double getHeight() const;

protected:
    bool step() override;

private:
    pros::Motor* motor;
    PID pid;
    ExitCondition settleExit;
    double target = 0;
    bool holdingTarget = false;
};

} // namespace xcore
