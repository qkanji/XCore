#include "main.h"

// This standalone demo project represents the real Override robot's drivetrain (not a
// separate bench-test chassis) - ports below are still placeholders pending final wiring,
// but the cartridge/gear ratio are the confirmed real values. Robot-specific auton/combo-
// function code still lives in OverrideBot-4862A, which depends on this XCore template.
using namespace xcore::config;

pros::Motor frontLeftMotor(1, pros::MotorGearset::green);
pros::Motor frontRightMotor(-2, pros::MotorGearset::green);
pros::Motor backLeftMotor(3, pros::MotorGearset::green);
pros::Motor backRightMotor(-4, pros::MotorGearset::green);
pros::Imu imu(10);

// Dedicated (non-drive) tracking wheels feeding LemLib's odometry - see xcore::XDrive.
// Ports/offsets are placeholders - see xcore::config for the values to tune.
pros::Rotation verticalTrackingSensor(11);
pros::Rotation horizontalTrackingSensor(12);
lemlib::TrackingWheel verticalTrackingWheel(&verticalTrackingSensor, VERTICAL_TRACKING_WHEEL_DIAMETER,
                                            VERTICAL_TRACKING_WHEEL_OFFSET);
lemlib::TrackingWheel horizontalTrackingWheel(&horizontalTrackingSensor, HORIZONTAL_TRACKING_WHEEL_DIAMETER,
                                              HORIZONTAL_TRACKING_WHEEL_OFFSET);

xcore::XDrive drive(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &imu, &verticalTrackingWheel,
                    &horizontalTrackingWheel, LATERAL_PID, LATERAL_SMALL_EXIT, LATERAL_LARGE_EXIT, ANGULAR_PID,
                    ANGULAR_SMALL_EXIT, ANGULAR_LARGE_EXIT);
xcore::RobotInterface robot(&drive);

// MCL is optional - 4 wall-facing distance sensors correcting LemLib's tracked pose.
// Ports/offsets are placeholders - see xcore::config for the values to measure/tune.
pros::Distance frontDistance(13);
pros::Distance backDistance(14);
pros::Distance leftDistance(15);
pros::Distance rightDistance(16);
xcore::MCL mcl({{&frontDistance, MCL_SENSOR_FRONT_X, MCL_SENSOR_FRONT_Y, MCL_SENSOR_FRONT_ANGLE},
                {&backDistance, MCL_SENSOR_BACK_X, MCL_SENSOR_BACK_Y, MCL_SENSOR_BACK_ANGLE},
                {&leftDistance, MCL_SENSOR_LEFT_X, MCL_SENSOR_LEFT_Y, MCL_SENSOR_LEFT_ANGLE},
                {&rightDistance, MCL_SENSOR_RIGHT_X, MCL_SENSOR_RIGHT_Y, MCL_SENSOR_RIGHT_ANGLE}},
               MCL_SETTINGS);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
	imu.reset();
	pros::delay(2000); // let the IMU finish calibrating

	if (MCL_ENABLED_BY_DEFAULT) mcl.enable(); // toggle anytime via mcl.enable()/mcl.disable()
}

/**
 * Runs while the robot is disabled.
 */
void disabled() {}

/**
 * Runs after initialize(), before autonomous, when connected to competition control.
 */
void competition_initialize() {}

/**
 * Demonstrates the chaining API: robot.do_(primary, triggers) blocks until `primary`
 * settles, firing each along() trigger at its progress threshold along the way.
 */
void autonomous() {
	robot.do_(robot.driveTo(24.0, 36.0, 270, 1500),
	          {xcore::along(0.5, [] { pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Halfway trigger fired!"); })});

	robot.do_(robot.driveTo(0.0, 0.0, 0, 1500));
}

/**
 * Runs the operator control code - plain field-centric arcade driving for bench testing.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		const double x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
		const double y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
		const double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

		drive.arcade(x, y, turn);

		pros::delay(10);
	}
}
