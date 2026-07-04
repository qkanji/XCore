#pragma once

#include "lemlib/chassis/odom.hpp"
#include "lemlib/util.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "xcore/types.hpp"
#include <atomic>
#include <cstdint>
#include <random>
#include <vector>

namespace xcore {

/**
 * A single distance sensor's mounting position, in robot-local-frame inches/degrees.
 *
 * `angleOffset` is the sensor's facing direction relative to the robot's own heading axis
 * (0 = facing the same way pose.theta=0 does; 90/180/-90 for sensors facing other sides).
 */
struct DistanceSensorMount {
    pros::Distance* sensor;
    float xOffset;
    float yOffset;
    float angleOffset;
};

/**
 * Tunable MCL parameters. A free-standing struct (rather than nested in MCL) so its default
 * member initializers are fully usable as a default argument on MCL's own constructor.
 */
struct MCLConfig {
    int particleCount = 200;
    float fieldSize = 144.0f; // inches, square field (12x12ft)
    float motionNoiseXY = 0.3f; // inches stddev added per update, on top of the measured odometry delta
    float motionNoiseTheta = 0.5f; // degrees stddev
    float sensorNoiseStddev = 2.0f; // inches stddev for the measurement (Gaussian weighting) model
    uint32_t updateIntervalMs = 25;
};

/**
 * Monte Carlo Localization: an optional, toggleable particle filter that corrects LemLib's
 * (odometry-tracked) global pose using 1-4 wall-facing distance sensors on a square field.
 *
 * This is an original implementation (not derived from any specific reference codebase) -
 * see the XCore README for the algorithmic references consulted during design. It assumes
 * a clear line of sight from each sensor to the nearest field wall; goals/obstacles in a
 * sensor's path will bias readings - a known limitation shared by all square-field-model MCL
 * implementations, not solved here.
 *
 * MCL runs its own background task independent of LemLib's odometry task, only touching the
 * shared global pose via lemlib::setPose() while enabled. When disabled, the task is idle and
 * XDrive/anything else reading getPose() sees pure odometry with no MCL correction.
 */
class MCL {
public:
    MCL(std::vector<DistanceSensorMount> sensors, MCLConfig config = MCLConfig());
    ~MCL();

    /** Seeds particles around the current lemlib::getPose() and starts correcting it. */
    void enable();

    /** Stops correcting the pose (odometry continues running independently via LemLib). */
    void disable();

    bool isEnabled() const;

private:
    struct Particle {
        float x, y, theta, weight;
    };

    void taskLoop();
    void motionUpdate();
    void measurementUpdate();
    void resample();
    float expectedDistance(const Particle& particle, const DistanceSensorMount& mount) const;

    std::vector<DistanceSensorMount> sensors;
    MCLConfig config;
    std::vector<Particle> particles;

    Pose lastOdomPose{0, 0, 0};
    std::atomic<bool> enabled{false};
    pros::Task* task = nullptr;
    std::mt19937 rng{std::random_device {}()};
};

} // namespace xcore
