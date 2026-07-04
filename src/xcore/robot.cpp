#include "xcore/robot.hpp"
#include "pros/rtos.hpp"

namespace xcore {

RobotInterface::RobotInterface(XDrive* drive) : drivePtr(drive) {}

DriveCommand* RobotInterface::driveTo(double x, double y, double theta, uint32_t timeout, double minSpeed,
                                       double maxSpeed) {
    auto* cmd = new DriveCommand(*drivePtr, x, y, theta, minSpeed, maxSpeed);
    cmd->timeout = timeout;
    return cmd;
}

void RobotInterface::do_(Command* primary, std::vector<Trigger> triggers) {
    const uint32_t start = pros::millis();
    primary->start();

    std::vector<bool> fired(triggers.size(), false);

    while (!primary->isSettled()) {
        if (primary->timeout > 0 && pros::millis() - start >= primary->timeout) {
            primary->cancel();
            break;
        }

        const double progress = primary->progress();
        for (std::size_t i = 0; i < triggers.size(); ++i) {
            if (!fired[i] && progress >= triggers[i].progress) {
                triggers[i].action();
                fired[i] = true;
            }
        }

        pros::delay(10);
    }

    delete primary;
}

} // namespace xcore
