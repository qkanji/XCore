#pragma once

#include "pros/adi.hpp"
#include "xcore/subsystem.hpp"
#include <cstdint>

namespace xcore {

/**
 * Thin extend/retract wrapper around a pneumatic piston, following the same busy-flag
 * convention as other subsystems so combo functions can wait on it via isBusy().
 */
class Pneumatics : public Subsystem {
public:
    explicit Pneumatics(pros::adi::Pneumatics* piston, uint32_t actuationTimeMs = 150);

    void extend();
    void retract();
    void set(bool extended);
    bool isExtended() const;

protected:
    bool step() override;

private:
    pros::adi::Pneumatics* piston;
    uint32_t actuationTimeMs;
    uint32_t actuatedAt = 0;
    bool moving = false;
};

} // namespace xcore
