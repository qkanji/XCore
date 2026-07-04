#pragma once

#include "xcore/mcl.hpp"
#include "xcore/types.hpp"

// Config.hpp/.cpp is the single place to tune the robot: drivetrain gearing, the 2 XDrive
// PIDs (+ their stop-detection thresholds), the Lift PID, tracking wheel geometry, and MCL.
// Nothing here should require touching subsystem logic to re-tune - only these values.
//
// NOTE: this file currently ships with placeholder values (clearly marked below) since it
// hasn't been tuned against real hardware yet. DRIVE_WHEEL_RPM is the one concrete figure
// confirmed for the real robot (green cartridge, 48:24 external gear).
namespace xcore::config {

// --- Drivetrain gearing --------------------------------------------------------------
// PROS has no native concept of "external gear ratio" - pros::Motor::get_actual_velocity()
// and get_position() only ever reflect the motor's own output shaft (post-cartridge,
// pre-external-gearing). Any code that needs real wheel RPM must compute it explicitly, so
// it's never silently wrong.
constexpr float DRIVE_CARTRIDGE_RPM = 200.0f; // green cartridge
constexpr float DRIVE_EXTERNAL_GEAR_RATIO = 48.0f / 24.0f; // driven:driving teeth - 2:1 speed increase
constexpr float DRIVE_WHEEL_RPM = DRIVE_CARTRIDGE_RPM * DRIVE_EXTERNAL_GEAR_RATIO; // 400 RPM

// --- Lateral (distance) PID + stop detection ------------------------------------------
// TODO: placeholder gains/thresholds - tune on the real robot.
extern const PID LATERAL_PID;
extern const ExitCondition LATERAL_SMALL_EXIT; // e.g. within 1in for 100ms
extern const ExitCondition LATERAL_LARGE_EXIT; // e.g. within 3in for 500ms

// --- Angular (heading) PID + stop detection --------------------------------------------
// TODO: placeholder gains/thresholds - tune on the real robot.
extern const PID ANGULAR_PID;
extern const ExitCondition ANGULAR_SMALL_EXIT; // e.g. within 1deg for 100ms
extern const ExitCondition ANGULAR_LARGE_EXIT; // e.g. within 3deg for 500ms

// --- Lift PID + stop detection (generic subsystem, only used if the robot has a lift) ---
// TODO: placeholder gains/threshold - tune on the real robot.
extern const PID LIFT_PID;
extern const ExitCondition LIFT_EXIT;

// --- Tracking wheels (feed XDrive's LemLib odometry - see xcore::XDrive) ---------------
// TODO: measure and update these on the real robot.
constexpr float VERTICAL_TRACKING_WHEEL_DIAMETER = 2.75f; // inches
constexpr float VERTICAL_TRACKING_WHEEL_OFFSET = 0.0f; // inches from the tracking center
constexpr float HORIZONTAL_TRACKING_WHEEL_DIAMETER = 2.75f; // inches
constexpr float HORIZONTAL_TRACKING_WHEEL_OFFSET = 0.0f; // inches from the tracking center

// --- MCL (optional - call mcl.enable()/disable() from robot code) ---------------------
// TODO: measure real sensor mounting offsets and tune noise/particle count on the robot.
constexpr bool MCL_ENABLED_BY_DEFAULT = false;
extern const MCLConfig MCL_SETTINGS;

constexpr float MCL_SENSOR_FRONT_X = 0.0f, MCL_SENSOR_FRONT_Y = 7.0f, MCL_SENSOR_FRONT_ANGLE = 0.0f;
constexpr float MCL_SENSOR_BACK_X = 0.0f, MCL_SENSOR_BACK_Y = -7.0f, MCL_SENSOR_BACK_ANGLE = 180.0f;
constexpr float MCL_SENSOR_LEFT_X = -7.0f, MCL_SENSOR_LEFT_Y = 0.0f, MCL_SENSOR_LEFT_ANGLE = -90.0f;
constexpr float MCL_SENSOR_RIGHT_X = 7.0f, MCL_SENSOR_RIGHT_Y = 0.0f, MCL_SENSOR_RIGHT_ANGLE = 90.0f;

} // namespace xcore::config
