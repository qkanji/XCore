#pragma once

#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/util.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "xcore/command.hpp"
#include "xcore/subsystem.hpp"
#include "xcore/types.hpp"
#include <cstdint>

namespace xcore {

/**
 * X-drive holonomic drivetrain subsystem.
 *
 * Owns the 4 drive motors and the IMU privately. Uses lemlib::PID for control output and
 * lemlib::ExitCondition for dual-threshold settle detection (e.g. within 1in for 100ms OR
 * 3in for 500ms) - see xcore::PID/ExitCondition in types.hpp. LemLib is a required
 * dependency of XCore for this reason (apply it before applying XCore).
 *
 * Pose tracking is delegated entirely to LemLib's own tracking-wheel + IMU odometry
 * (lemlib::setSensors()/init()/getPose()/setPose(), which are free functions operating on
 * process-global state - LemLib's odometry is a singleton, so only construct one XDrive).
 * A dedicated vertical + horizontal tracking wheel is required (not derived from the drive
 * motors), since powered X-drive rollers slip too much to be a reliable position source.
 * Internally, XDrive builds a throwaway "substitute" tracking wheel (backed by a MotorGroup
 * wrapping frontLeft) purely to satisfy LemLib's non-null check on the second vertical
 * wheel slot - this forces LemLib to use the IMU for heading (its next priority) rather
 * than a second real vertical wheel we don't have; its distance-traveled value is never
 * actually read by LemLib's tracking math once a horizontal wheel + IMU are present.
 */
class XDrive : public Subsystem {
public:
    /**
     * @param verticalWheel / horizontalWheel dedicated (non-drive) tracking wheels feeding
     * LemLib's odometry - see lemlib::TrackingWheel for offset/gear-ratio conventions.
     * @param lateralPID / turnPID control the distance and heading error respectively during
     * directToPose().
     * @param lateralSmallExit/lateralLargeExit and angularSmallExit/angularLargeExit implement
     * the dual-threshold stop detection (e.g. within 1in for 100ms OR 3in for 500ms; within
     * 1deg for 100ms OR 3deg for 500ms).
     */
    XDrive(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight,
           pros::Imu* imu, lemlib::TrackingWheel* verticalWheel, lemlib::TrackingWheel* horizontalWheel,
           PID lateralPID, ExitCondition lateralSmallExit, ExitCondition lateralLargeExit, PID turnPID,
           ExitCondition angularSmallExit, ExitCondition angularLargeExit);

    /** Async: begins driving toward (x, y, theta). Returns immediately. */
    void directToPose(double x, double y, double theta, double minSpeed = 0, double maxSpeed = 127);

    /** Sync: directToPose(...) followed by waitUntilDone(). */
    void directToPoseSync(double x, double y, double theta, double minSpeed = 0, double maxSpeed = 127);

    /** Field-centric manual control from normalized joystick inputs in [-1, 1]. */
    void arcade(double x, double y, double turn);

    /** Thin wrapper over LemLib's global lemlib::getPose()/setPose(). */
    Pose getPose() const;
    void setPose(const Pose& pose);

    void stop();

protected:
    bool step() override;

private:
    void driveMotors(double power, double localAngleDeg, double turn, double minSpeed, double maxSpeed);

    pros::Motor* frontLeft;
    pros::Motor* frontRight;
    pros::Motor* backLeft;
    pros::Motor* backRight;
    pros::Imu* imu;

    // LemLib odometry compatibility shim - see class doc comment above. Never read for its
    // actual distance-traveled value.
    pros::MotorGroup verticalSubstituteMotors;
    lemlib::TrackingWheel verticalSubstitute;

    PID lateralPID;
    ExitCondition lateralSmallExit;
    ExitCondition lateralLargeExit;
    PID turnPID;
    ExitCondition angularSmallExit;
    ExitCondition angularLargeExit;

    Pose target{0, 0, 0};
    double minSpeed = 0;
    double maxSpeed = 127;
    bool holdingTarget = false;
};

/** Command wrapper binding a directToPose call to the Command interface used by RobotInterface::do_(). */
class DriveCommand : public Command {
public:
    DriveCommand(XDrive& drive, double x, double y, double theta, double minSpeed = 0, double maxSpeed = 127);

    void start() override;
    bool isSettled() const override;
    void cancel() override;
    double progress() const override;

    /**
     * Marks this as a non-terminal waypoint in a chained S-curve: isSettled() switches from
     * XDrive's full stop-settle to a simple proximity-radius check, so the next queued
     * movement can begin without the robot ever decelerating below minSpeed.
     */
    Command* chain(double minSpeed = 50, double proximityRadius = 3.0) override;

private:
    XDrive& drive;
    double x, y, theta, minSpeed, maxSpeed;
    double proximityRadius = 3.0;
    Pose startPose{0, 0, 0};
    bool chained = false;
};

} // namespace xcore
