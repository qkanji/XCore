#include "xcore/mcl.hpp"
#include <cmath>
#include <limits>
#include <utility>

namespace xcore {

MCL::MCL(std::vector<DistanceSensorMount> sensors, MCLConfig config) : sensors(std::move(sensors)), config(config) {
    task = new pros::Task([this] { taskLoop(); });
}

MCL::~MCL() {
    if (task != nullptr) {
        task->remove();
        delete task;
    }
}

void MCL::enable() {
    const Pose current = lemlib::getPose();
    lastOdomPose = current;

    particles.assign(config.particleCount, Particle{current.x, current.y, current.theta, 1.0f / config.particleCount});
    enabled = true;
}

void MCL::disable() { enabled = false; }

bool MCL::isEnabled() const { return enabled.load(); }

void MCL::taskLoop() {
    while (true) {
        if (enabled) {
            motionUpdate();
            measurementUpdate();
            resample();

            // Circular mean for theta avoids wraparound bias (e.g. averaging 359 and 1
            // degrees should give 0, not 180).
            float sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
            for (const Particle& p : particles) {
                sumX += p.x;
                sumY += p.y;
                sumSin += std::sin(lemlib::degToRad(p.theta));
                sumCos += std::cos(lemlib::degToRad(p.theta));
            }
            const float n = static_cast<float>(particles.size());
            const Pose corrected(sumX / n, sumY / n, lemlib::radToDeg(std::atan2(sumSin / n, sumCos / n)));
            lemlib::setPose(corrected);
        }
        pros::delay(config.updateIntervalMs);
    }
}

void MCL::motionUpdate() {
    const Pose current = lemlib::getPose();
    const float dx = current.x - lastOdomPose.x;
    const float dy = current.y - lastOdomPose.y;
    const float dtheta = lemlib::angleError(current.theta, lastOdomPose.theta, false);
    lastOdomPose = current;

    std::normal_distribution<float> xyNoise(0.0f, config.motionNoiseXY);
    std::normal_distribution<float> thetaNoise(0.0f, config.motionNoiseTheta);

    // Heading is trusted almost entirely to the IMU (via LemLib's odometry) rather than
    // maintained independently per-particle - only position (x, y) is meaningfully
    // re-estimated by the distance-sensor measurement step below. This matches how most
    // VRC-scale MCL implementations handle holonomic/tank robots with a reliable IMU.
    for (Particle& p : particles) {
        p.x += dx + xyNoise(rng);
        p.y += dy + xyNoise(rng);
        p.theta += dtheta + thetaNoise(rng);
    }
}

float MCL::expectedDistance(const Particle& particle, const DistanceSensorMount& mount) const {
    const Pose offset = Pose(mount.xOffset, mount.yOffset).rotate(lemlib::degToRad(particle.theta));
    const float sensorX = particle.x + offset.x;
    const float sensorY = particle.y + offset.y;
    const float globalAngle = lemlib::degToRad(particle.theta + mount.angleOffset);

    const float half = config.fieldSize / 2.0f;
    const float dirX = std::cos(globalAngle);
    const float dirY = std::sin(globalAngle);

    float best = std::numeric_limits<float>::max();
    if (dirX > 1e-6f) best = std::min(best, (half - sensorX) / dirX);
    else if (dirX < -1e-6f) best = std::min(best, (-half - sensorX) / dirX);
    if (dirY > 1e-6f) best = std::min(best, (half - sensorY) / dirY);
    else if (dirY < -1e-6f) best = std::min(best, (-half - sensorY) / dirY);

    return std::max(0.0f, best);
}

void MCL::measurementUpdate() {
    for (Particle& p : particles) {
        for (const DistanceSensorMount& mount : sensors) {
            const int32_t rawMm = mount.sensor->get_distance();
            if (rawMm < 0) continue; // sensor error/out of range - skip this reading

            const float actualIn = rawMm / 25.4f;
            const float expectedIn = expectedDistance(p, mount);
            const float diff = actualIn - expectedIn;
            const float likelihood =
                std::exp(-0.5f * (diff * diff) / (config.sensorNoiseStddev * config.sensorNoiseStddev));

            // Small floor prevents a single unlucky/obstructed reading from zeroing out an
            // otherwise-good particle's weight entirely.
            p.weight *= std::max(likelihood, 1e-6f);
        }
    }
}

void MCL::resample() {
    float totalWeight = 0;
    for (const Particle& p : particles) totalWeight += p.weight;
    if (totalWeight <= 0) return; // degenerate case - keep the existing particle set

    const std::size_t n = particles.size();
    std::vector<Particle> resampled;
    resampled.reserve(n);

    // Low-variance (systematic) resampling.
    std::uniform_real_distribution<float> startDist(0.0f, 1.0f / n);
    float r = startDist(rng);
    float c = particles[0].weight / totalWeight;
    std::size_t i = 0;

    for (std::size_t m = 0; m < n; ++m) {
        const float u = r + static_cast<float>(m) / n;
        while (u > c && i < n - 1) {
            i++;
            c += particles[i].weight / totalWeight;
        }
        resampled.push_back(particles[i]);
        resampled.back().weight = 1.0f / n;
    }

    particles = std::move(resampled);
}

} // namespace xcore
