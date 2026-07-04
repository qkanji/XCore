#include "xcore/xdrive.hpp"
#include <algorithm>
#include <cmath>

namespace xcore {

XDrive::XDrive(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight,
               pros::Imu* imu, lemlib::TrackingWheel* verticalWheel, lemlib::TrackingWheel* horizontalWheel,
               PID lateralPID, ExitCondition lateralSmallExit, ExitCondition lateralLargeExit, PID turnPID,
               ExitCondition angularSmallExit, ExitCondition angularLargeExit)
    : frontLeft(frontLeft),
      frontRight(frontRight),
      backLeft(backLeft),
      backRight(backRight),
      imu(imu),
      verticalSubstituteMotors({frontLeft->get_port()}),
      verticalSubstitute(&verticalSubstituteMotors, 2.75, 0, 200),
      lateralPID(lateralPID),
      lateralSmallExit(lateralSmallExit),
      lateralLargeExit(lateralLargeExit),
      turnPID(turnPID),
      angularSmallExit(angularSmallExit),
      angularLargeExit(angularLargeExit) {
    verticalWheel->reset();
    horizontalWheel->reset();

    // Wire dedicated tracking wheels + IMU into LemLib's (singleton) odometry system,
    // bypassing its tank-drive Drivetrain fallback entirely - see the class doc comment.
    lemlib::OdomSensors sensors(verticalWheel, &verticalSubstitute, horizontalWheel, nullptr, imu);
    lemlib::Drivetrain unusedDrivetrain(nullptr, nullptr, 0, 0, 0, 0); // never read by lemlib::update()
    lemlib::setSensors(sensors, unusedDrivetrain);
    lemlib::init();

    startTask();
}

void XDrive::directToPose(double x, double y, double theta, double minSpeedIn, double maxSpeedIn) {
    target = Pose(x, y, theta);
    minSpeed = minSpeedIn;
    maxSpeed = maxSpeedIn;
    lateralPID.reset();
    turnPID.reset();
    lateralSmallExit.reset();
    lateralLargeExit.reset();
    angularSmallExit.reset();
    angularLargeExit.reset();
    holdingTarget = true;
    markBusy();
}

void XDrive::directToPoseSync(double x, double y, double theta, double minSpeedIn, double maxSpeedIn) {
    directToPose(x, y, theta, minSpeedIn, maxSpeedIn);
    waitUntilDone();
}

void XDrive::driveMotors(double power, double localAngleDeg, double turn, double minSpeedIn, double maxSpeedIn) {
    const float localAngle = lemlib::degToRad(localAngleDeg);
    const double sign = power < 0 ? -1.0 : 1.0;
    const double clampedMagnitude = std::clamp(std::fabs(power), minSpeedIn, maxSpeedIn);
    const double clampedPower = sign * clampedMagnitude;

    const double fl = clampedPower * std::sin(localAngle + M_PI / 4) + turn;
    const double fr = clampedPower * std::cos(localAngle + M_PI / 4) - turn;
    const double bl = clampedPower * std::cos(localAngle + M_PI / 4) + turn;
    const double br = clampedPower * std::sin(localAngle + M_PI / 4) - turn;

    frontLeft->move(fl);
    frontRight->move(fr);
    backLeft->move(bl);
    backRight->move(br);
}

void XDrive::arcade(double x, double y, double turn) {
    holdingTarget = false;
    const Pose pose = getPose();
    const float globalAngle = std::atan2(y, x); // radians
    const float localAngle = globalAngle - lemlib::degToRad(pose.theta);
    const double power = std::min(1.0, std::sqrt(x * x + y * y)) * 127.0;
    driveMotors(power, lemlib::radToDeg(localAngle), turn * 127.0, 0, 127);
}

Pose XDrive::getPose() const { return lemlib::getPose(); }

void XDrive::setPose(const Pose& newPose) { lemlib::setPose(newPose); }

void XDrive::stop() {
    holdingTarget = false;
    frontLeft->move(0);
    frontRight->move(0);
    backLeft->move(0);
    backRight->move(0);
}

bool XDrive::step() {
    if (!holdingTarget) return true;

    const Pose pose = getPose();
    const float dist = pose.distance(target);
    const float angleDiff = lemlib::angleError(target.theta, pose.theta, false); // degrees, shortest path

    const float drivePower = lateralPID.update(dist);
    const float turn = turnPID.update(angleDiff);

    const float globalAngle = pose.angle(target); // radians
    const float localAngle = globalAngle - lemlib::degToRad(pose.theta);

    driveMotors(drivePower, lemlib::radToDeg(localAngle), turn, minSpeed, maxSpeed);

    const bool lateralSettled = lateralSmallExit.update(dist) || lateralLargeExit.update(dist);
    const bool angularSettled =
        angularSmallExit.update(std::fabs(angleDiff)) || angularLargeExit.update(std::fabs(angleDiff));
    const bool reached = lateralSettled && angularSettled;

    if (reached) {
        holdingTarget = false;
        stop();
    }
    return reached;
}

// --- DriveCommand -----------------------------------------------------------------------

DriveCommand::DriveCommand(XDrive& drive, double x, double y, double theta, double minSpeed, double maxSpeed)
    : drive(drive), x(x), y(y), theta(theta), minSpeed(minSpeed), maxSpeed(maxSpeed) {}

void DriveCommand::start() {
    startPose = drive.getPose();
    drive.directToPose(x, y, theta, minSpeed, maxSpeed);
}

bool DriveCommand::isSettled() const {
    if (chained) {
        // Proximity-only check: lets the caller move on to the next queued command before
        // XDrive's own PID/ExitCondition settle logic would fire, so the robot never
        // decelerates to a stop mid S-curve.
        return drive.getPose().distance(Pose(x, y, theta)) <= proximityRadius;
    }
    return !drive.isBusy();
}

void DriveCommand::cancel() { drive.stop(); }

double DriveCommand::progress() const { return drive.getPose().distance(startPose); }

Command* DriveCommand::chain(double minSpeedIn, double proximityRadiusIn) {
    chained = true;
    minSpeed = minSpeedIn;
    proximityRadius = proximityRadiusIn;
    return this;
}

} // namespace xcore
