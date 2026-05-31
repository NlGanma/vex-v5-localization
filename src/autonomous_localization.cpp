#include "autonomous_localization.hpp"

#include "robot.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace autonomous_localization {
namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kPiOver2 = 1.5707963267948966f;
constexpr std::uint32_t kRelocalizeTimeoutMs = 3500;
constexpr float kRelocalizeMinConfidence = 0.22f;
constexpr float kRelocalizeWeakConfidence = 0.06f;
constexpr float kRelocalizeWeakMaxVarXY = 64.0f;
constexpr float kRelocalizeWeakMaxVarTheta = 0.05f;
constexpr int kRelocalizeMinSensors = 2;
constexpr float kRelocalizeStartupMaxRange = 96.0f;
constexpr int kRelocalizeStartupMinConfidence = 12;
constexpr int kRelocalizeSnapshotCount = 3;
constexpr std::uint32_t kRelocalizeSnapshotSpacingMs = 20;
constexpr int kRelocalizeCoarseHeadingHypotheses = 16;
constexpr int kRelocalizeRefineHeadingHypotheses = 5;
constexpr int kRelocalizeIterationsPerHypothesis = 6;
constexpr float kDirectWallPriorWeight = 0.05f;
constexpr float kDirectWallIgnoredSensorPenalty = 2.5f;
constexpr float kDirectWallOutlierPenalty = 4.0f;
constexpr float kRelocalizeMaxMeanResidual = 5.0f;
constexpr float kRelocalizeMaxResidual = 12.0f;
constexpr int kDirectWallCommitMinSensors = 3;
constexpr int kDirectWallTwoAxisMinSensors = 2;
constexpr float kDirectWallTwoAxisMaxMeanResidual = 1.5f;
constexpr float kDirectWallTwoAxisMaxResidual = 3.0f;
constexpr int kRelocalizeCommitMinSensors = 3;
// At competition start the IMU heading is a strong, drift-free prior. Use it to
// break the near-square field's 90/180 deg wall-distance aliasing: penalize
// candidate headings that disagree with the IMU, and refuse to commit a
// wall_direct solve whose snapped cardinal fights the IMU beyond this window.
constexpr float kRelocalizeHeadingPriorWeightPerRad = 6.0f;
constexpr float kRelocalizeMaxHeadingDisagreementRad = 0.6981317f; // 40 deg
constexpr float kRelocalizeHeadingTiebreakRad = 0.3490659f;        // 20 deg

enum class WallAxisDirection { PosX, NegX, PosY, NegY, Unknown };

struct DirectWallSolveCandidate {
    bool valid = false;
    lemlib::Pose pose {0, 0, 0};
    float meanAbsResidual = 0.0f;
    float maxAbsResidual = 0.0f;
    float selectionScore = 0.0f;
    int usedSensors = 0;
    int scoredSensors = 0;
    int inlierSensors = 0;
    int xSensors = 0;
    int ySensors = 0;
};

struct RelocalizationCandidate {
    lemlib::localization::MCLMeasurement measurement {};
    float seedHeading = 0.0f;
    int iterations = 0;
    bool hasMeasurement = false;
};

struct PoseResidualSummary {
    float meanAbsResidual = 0.0f;
    float maxAbsResidual = 0.0f;
    int scoredSensors = 0;
    int inlierSensors = 0;
};

SensorSnapshot captureSensor(pros::Distance& sensor) {
    SensorSnapshot snapshot {};
    const std::int32_t distanceMm = sensor.get_distance();
    if (distanceMm <= 0 || distanceMm >= 9000) return snapshot;

    snapshot.distance = static_cast<float>(distanceMm) / 25.4f;
    if (distanceMm >= 200) snapshot.confidence = sensor.get_confidence();
    return snapshot;
}

template <size_t N>
SensorSnapshot combineSensorSnapshots(const std::array<SensorSnapshot, N>& samples) {
    SensorSnapshot combined {};
    std::array<float, N> distances {};
    size_t distanceCount = 0;

    for (const auto& sample : samples) {
        if (sample.distance > 0.0f) distances[distanceCount++] = sample.distance;
        if (sample.confidence > combined.confidence) combined.confidence = sample.confidence;
    }

    if (distanceCount == 0) return combined;

    std::sort(distances.begin(), distances.begin() + static_cast<std::ptrdiff_t>(distanceCount));
    if (distanceCount % 2 == 1) combined.distance = distances[distanceCount / 2];
    else combined.distance = 0.5f * (distances[distanceCount / 2 - 1] + distances[distanceCount / 2]);

    return combined;
}

DistanceSnapshot captureStableDistances() {
    std::array<DistanceSnapshot, kRelocalizeSnapshotCount> snapshots {};
    for (int i = 0; i < kRelocalizeSnapshotCount; ++i) {
        snapshots[static_cast<size_t>(i)] = DistanceSnapshot {
            captureSensor(distFront),
            captureSensor(distRight),
            captureSensor(distBack),
            captureSensor(distLeft),
        };
        if (i + 1 < kRelocalizeSnapshotCount) pros::delay(kRelocalizeSnapshotSpacingMs);
    }

    std::array<SensorSnapshot, kRelocalizeSnapshotCount> front {};
    std::array<SensorSnapshot, kRelocalizeSnapshotCount> right {};
    std::array<SensorSnapshot, kRelocalizeSnapshotCount> back {};
    std::array<SensorSnapshot, kRelocalizeSnapshotCount> left {};
    for (int i = 0; i < kRelocalizeSnapshotCount; ++i) {
        front[static_cast<size_t>(i)] = snapshots[static_cast<size_t>(i)].front;
        right[static_cast<size_t>(i)] = snapshots[static_cast<size_t>(i)].right;
        back[static_cast<size_t>(i)] = snapshots[static_cast<size_t>(i)].back;
        left[static_cast<size_t>(i)] = snapshots[static_cast<size_t>(i)].left;
    }

    return DistanceSnapshot {
        combineSensorSnapshots(front),
        combineSensorSnapshots(right),
        combineSensorSnapshots(back),
        combineSensorSnapshots(left),
    };
}

std::array<lemlib::localization::SensorObservation, 4> makeSensorObservations(const DistanceSnapshot& snapshot) {
    const std::array<SensorSnapshot, 4> samples {snapshot.front, snapshot.right, snapshot.back, snapshot.left};
    std::array<lemlib::localization::SensorObservation, 4> observations {};
    for (size_t i = 0; i < samples.size(); ++i) {
        if (samples[i].distance <= 0.0f) continue;
        observations[i].available = true;
        observations[i].distance = samples[i].distance;
        observations[i].confidence = samples[i].confidence;
    }
    return observations;
}

float wrapRadians(float radians) {
    while (radians > kPi) radians -= 2.0f * kPi;
    while (radians <= -kPi) radians += 2.0f * kPi;
    return radians;
}

float currentHeadingRadians() {
    const double imuRotationDeg = imu.get_rotation();
    if (std::isfinite(imuRotationDeg)) return wrapRadians(lemlib::degToRad(static_cast<float>(imuRotationDeg)));
    return wrapRadians(chassis.getPose(true).theta);
}

bool sensorSnapshotUsable(const SensorSnapshot& snapshot, const lemlib::localization::SensorConfig& sensor) {
    if (snapshot.distance < sensor.minRange || snapshot.distance > sensor.maxRange) return false;
    if (snapshot.confidence >= 0 && snapshot.confidence < sensor.minConfidence) return false;
    return snapshot.distance > 0.0f;
}

int countUsableDistanceSnapshots(const lemlib::localization::LocalizationConfig& config,
                                 const DistanceSnapshot& snapshot) {
    const std::array<SensorSnapshot, 4> samples {snapshot.front, snapshot.right, snapshot.back, snapshot.left};
    int count = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
        if (config.sensors[i].sensor == nullptr) continue;
        if (sensorSnapshotUsable(samples[i], config.sensors[i])) ++count;
    }
    return count;
}

float raycastCircle(float sx, float sy, float dirX, float dirY, float cx, float cy, float radius) {
    const float dx = sx - cx;
    const float dy = sy - cy;
    const float a = dirX * dirX + dirY * dirY;
    const float b = 2.0f * (dx * dirX + dy * dirY);
    const float c = dx * dx + dy * dy - radius * radius;
    const float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) return -1.0f;
    const float sqrtDisc = std::sqrt(disc);
    const float inv2A = 1.0f / (2.0f * a);
    const float t1 = (-b - sqrtDisc) * inv2A;
    const float t2 = (-b + sqrtDisc) * inv2A;
    if (t1 > 0.0f && t2 > 0.0f) return std::min(t1, t2);
    if (t1 > 0.0f) return t1;
    if (t2 > 0.0f) return t2;
    return -1.0f;
}

float raycastRect(float sx, float sy, float dirX, float dirY, float minX, float maxX, float minY, float maxY) {
    constexpr float eps = 1e-6f;
    float tMin = -1.0e9f;
    float tMax = 1.0e9f;

    if (std::fabs(dirX) < eps) {
        if (sx < minX || sx > maxX) return -1.0f;
    } else {
        const float tx1 = (minX - sx) / dirX;
        const float tx2 = (maxX - sx) / dirX;
        tMin = std::max(tMin, std::min(tx1, tx2));
        tMax = std::min(tMax, std::max(tx1, tx2));
    }

    if (std::fabs(dirY) < eps) {
        if (sy < minY || sy > maxY) return -1.0f;
    } else {
        const float ty1 = (minY - sy) / dirY;
        const float ty2 = (maxY - sy) / dirY;
        tMin = std::max(tMin, std::min(ty1, ty2));
        tMax = std::min(tMax, std::max(ty1, ty2));
    }

    if (tMax < 0.0f || tMin > tMax) return -1.0f;
    if (tMin >= 0.0f) return tMin;
    if (tMax >= 0.0f) return tMax;
    return -1.0f;
}

float expectedDistance(const lemlib::localization::LocalizationConfig& config, const lemlib::Pose& pose,
                       const lemlib::localization::SensorConfig& sensor) {
    const float sinTheta = std::sin(pose.theta);
    const float cosTheta = std::cos(pose.theta);
    const float sx = pose.x + sensor.dx * cosTheta + sensor.dy * sinTheta;
    const float sy = pose.y - sensor.dx * sinTheta + sensor.dy * cosTheta;
    const float heading = wrapRadians(pose.theta + sensor.dtheta);
    const float dirX = std::sin(heading);
    const float dirY = std::cos(heading);
    constexpr float eps = 1e-6f;
    float best = 1.0e9f;

    auto consider = [&](float distance) {
        if (distance > 0.0f && distance < best) best = distance;
    };

    if (std::fabs(dirX) > eps) {
        const float t1 = (config.field.minX - sx) / dirX;
        const float y1 = sy + t1 * dirY;
        if (t1 > 0.0f && y1 >= config.field.minY && y1 <= config.field.maxY) consider(t1);

        const float t2 = (config.field.maxX - sx) / dirX;
        const float y2 = sy + t2 * dirY;
        if (t2 > 0.0f && y2 >= config.field.minY && y2 <= config.field.maxY) consider(t2);
    }

    if (std::fabs(dirY) > eps) {
        const float t3 = (config.field.minY - sy) / dirY;
        const float x3 = sx + t3 * dirX;
        if (t3 > 0.0f && x3 >= config.field.minX && x3 <= config.field.maxX) consider(t3);

        const float t4 = (config.field.maxY - sy) / dirY;
        const float x4 = sx + t4 * dirX;
        if (t4 > 0.0f && x4 >= config.field.minX && x4 <= config.field.maxX) consider(t4);
    }

    for (const auto& obstacle : config.field.obstacles) {
        float hit = -1.0f;
        if (obstacle.type == lemlib::localization::FieldConfig::Obstacle::Type::Circle) {
            hit = raycastCircle(sx, sy, dirX, dirY, obstacle.x, obstacle.y, obstacle.radius);
        } else {
            hit = raycastRect(sx, sy, dirX, dirY, obstacle.x - obstacle.halfW, obstacle.x + obstacle.halfW,
                              obstacle.y - obstacle.halfH, obstacle.y + obstacle.halfH);
        }
        consider(hit);
    }

    return best >= 1.0e8f ? -1.0f : best;
}

PoseResidualSummary scorePoseResiduals(const lemlib::localization::LocalizationConfig& config,
                                       const DistanceSnapshot& snapshot, const lemlib::Pose& pose) {
    const std::array<SensorSnapshot, 4> samples {snapshot.front, snapshot.right, snapshot.back, snapshot.left};
    PoseResidualSummary out {};
    float residualSum = 0.0f;
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& sensor = config.sensors[i];
        const auto& sample = samples[i];
        if (sensor.sensor == nullptr || !sensorSnapshotUsable(sample, sensor)) continue;
        const float predicted = expectedDistance(config, pose, sensor);
        if (predicted <= 0.0f) continue;
        const float residual = std::fabs(predicted - sample.distance);
        residualSum += residual;
        out.maxAbsResidual = std::max(out.maxAbsResidual, residual);
        ++out.scoredSensors;
        if (residual <= config.mcl.outlierThreshold) ++out.inlierSensors;
    }
    if (out.scoredSensors > 0) out.meanAbsResidual = residualSum / static_cast<float>(out.scoredSensors);
    return out;
}

bool residualsAccepted(const PoseResidualSummary& residuals, int minSensors,
                       float maxMeanResidual = kRelocalizeMaxMeanResidual,
                       float maxResidual = kRelocalizeMaxResidual) {
    return residuals.scoredSensors >= minSensors &&
           residuals.inlierSensors >= minSensors &&
           residuals.meanAbsResidual <= maxMeanResidual &&
           residuals.maxAbsResidual <= maxResidual;
}

WallAxisDirection classifyWallAxisDirection(float heading) {
    const float normalized = wrapRadians(heading);
    const float absHeading = std::fabs(normalized);
    if (std::fabs(normalized) <= lemlib::degToRad(30.0f)) return WallAxisDirection::PosY;
    if (std::fabs(absHeading - kPi) <= lemlib::degToRad(30.0f)) return WallAxisDirection::NegY;
    if (std::fabs(normalized - kPiOver2) <= lemlib::degToRad(30.0f)) return WallAxisDirection::PosX;
    if (std::fabs(normalized + kPiOver2) <= lemlib::degToRad(30.0f)) return WallAxisDirection::NegX;
    return WallAxisDirection::Unknown;
}

DirectWallSolveCandidate solveDirectWallPose(const lemlib::localization::LocalizationConfig& config,
                                             const DistanceSnapshot& snapshot, float heading,
                                             const lemlib::Pose& priorPose) {
    const std::array<SensorSnapshot, 4> samples {snapshot.front, snapshot.right, snapshot.back, snapshot.left};
    std::array<WallAxisDirection, 4> axes {};
    int usableAxisSensors = 0;

    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& sensor = config.sensors[i];
        const auto& sample = samples[i];
        if (sensor.sensor == nullptr || !sensorSnapshotUsable(sample, sensor)) continue;
        const WallAxisDirection axis = classifyWallAxisDirection(wrapRadians(heading + sensor.dtheta));
        axes[i] = axis;
        if (axis != WallAxisDirection::Unknown) ++usableAxisSensors;
    }

    DirectWallSolveCandidate best {};
    best.pose.theta = wrapRadians(heading);
    best.selectionScore = 1.0e9f;

    for (int mask = 1; mask < (1 << static_cast<int>(samples.size())); ++mask) {
        DirectWallSolveCandidate candidate {};
        candidate.pose.theta = wrapRadians(heading);
        float xSum = 0.0f;
        float ySum = 0.0f;

        for (size_t i = 0; i < samples.size(); ++i) {
            if ((mask & (1 << static_cast<int>(i))) == 0) continue;
            const auto& sensor = config.sensors[i];
            const auto& sample = samples[i];
            const WallAxisDirection axis = axes[i];
            if (sensor.sensor == nullptr || !sensorSnapshotUsable(sample, sensor) || axis == WallAxisDirection::Unknown) {
                continue;
            }

            const float sinTheta = std::sin(candidate.pose.theta);
            const float cosTheta = std::cos(candidate.pose.theta);
            const float sensorGlobalX = sensor.dx * cosTheta + sensor.dy * sinTheta;
            const float sensorGlobalY = -sensor.dx * sinTheta + sensor.dy * cosTheta;
            const float sensorHeading = candidate.pose.theta + sensor.dtheta;
            const float rayDirX = std::sin(sensorHeading);
            const float rayDirY = std::cos(sensorHeading);

            switch (axis) {
                case WallAxisDirection::PosX:
                    xSum += config.field.maxX - rayDirX * sample.distance - sensorGlobalX;
                    ++candidate.xSensors;
                    ++candidate.usedSensors;
                    break;
                case WallAxisDirection::NegX:
                    xSum += config.field.minX - rayDirX * sample.distance - sensorGlobalX;
                    ++candidate.xSensors;
                    ++candidate.usedSensors;
                    break;
                case WallAxisDirection::PosY:
                    ySum += config.field.maxY - rayDirY * sample.distance - sensorGlobalY;
                    ++candidate.ySensors;
                    ++candidate.usedSensors;
                    break;
                case WallAxisDirection::NegY:
                    ySum += config.field.minY - rayDirY * sample.distance - sensorGlobalY;
                    ++candidate.ySensors;
                    ++candidate.usedSensors;
                    break;
                case WallAxisDirection::Unknown:
                    break;
            }
        }

        if (candidate.xSensors == 0 || candidate.ySensors == 0 || candidate.usedSensors < 2) continue;

        candidate.pose.x = xSum / static_cast<float>(candidate.xSensors);
        candidate.pose.y = ySum / static_cast<float>(candidate.ySensors);

        float residualSum = 0.0f;
        for (size_t i = 0; i < samples.size(); ++i) {
            const auto& sensor = config.sensors[i];
            const auto& sample = samples[i];
            const WallAxisDirection axis = axes[i];
            if (sensor.sensor == nullptr || !sensorSnapshotUsable(sample, sensor) || axis == WallAxisDirection::Unknown) {
                continue;
            }

            const float sinTheta = std::sin(candidate.pose.theta);
            const float cosTheta = std::cos(candidate.pose.theta);
            const float sensorX = candidate.pose.x + sensor.dx * cosTheta + sensor.dy * sinTheta;
            const float sensorY = candidate.pose.y - sensor.dx * sinTheta + sensor.dy * cosTheta;
            const float sensorHeading = candidate.pose.theta + sensor.dtheta;
            const float rayDirX = std::sin(sensorHeading);
            const float rayDirY = std::cos(sensorHeading);

            float predicted = -1.0f;
            switch (axis) {
                case WallAxisDirection::PosX: predicted = (config.field.maxX - sensorX) / rayDirX; break;
                case WallAxisDirection::NegX: predicted = (config.field.minX - sensorX) / rayDirX; break;
                case WallAxisDirection::PosY: predicted = (config.field.maxY - sensorY) / rayDirY; break;
                case WallAxisDirection::NegY: predicted = (config.field.minY - sensorY) / rayDirY; break;
                case WallAxisDirection::Unknown: break;
            }
            if (predicted <= 0.0f) continue;

            const float residual = std::fabs(predicted - sample.distance);
            residualSum += residual;
            candidate.maxAbsResidual = std::max(candidate.maxAbsResidual, residual);
            ++candidate.scoredSensors;
            if (residual <= config.mcl.outlierThreshold) ++candidate.inlierSensors;
        }

        if (candidate.scoredSensors == 0 || candidate.inlierSensors < kRelocalizeMinSensors) continue;

        candidate.meanAbsResidual = residualSum / static_cast<float>(candidate.scoredSensors);
        const bool insideField = candidate.pose.x >= config.field.minX && candidate.pose.x <= config.field.maxX &&
                                 candidate.pose.y >= config.field.minY && candidate.pose.y <= config.field.maxY;
        if (!insideField) continue;

        const float priorDistance = candidate.pose.distance(priorPose);
        const int ignoredSensors = std::max(0, usableAxisSensors - candidate.usedSensors);
        const int outlierSensors = std::max(0, candidate.scoredSensors - candidate.inlierSensors);
        candidate.selectionScore = candidate.meanAbsResidual + kDirectWallPriorWeight * priorDistance +
                                   kDirectWallIgnoredSensorPenalty * static_cast<float>(ignoredSensors) +
                                   kDirectWallOutlierPenalty * static_cast<float>(outlierSensors);
        candidate.valid = true;
        if (!best.valid || candidate.selectionScore < best.selectionScore) best = candidate;
    }

    return best;
}

bool relocalizationAccepted(const lemlib::localization::MCLMeasurement& measurement,
                            const lemlib::localization::LocalizationConfig& config) {
    return measurement.valid && measurement.activeSensors >= kRelocalizeCommitMinSensors &&
           measurement.confidence >= kRelocalizeMinConfidence &&
           measurement.covariance.m[0][0] <= config.fusion.maxVarXY &&
           measurement.covariance.m[1][1] <= config.fusion.maxVarXY &&
           measurement.covariance.m[2][2] <= config.fusion.maxVarTheta;
}

bool relocalizationWeakAccepted(const lemlib::localization::MCLMeasurement& measurement,
                                const lemlib::localization::LocalizationConfig& config) {
    if (!measurement.valid || measurement.activeSensors < kRelocalizeCommitMinSensors) return false;

    const float maxVarXY = std::min(config.fusion.maxVarXY, kRelocalizeWeakMaxVarXY);
    const float maxVarTheta = std::min(config.fusion.maxVarTheta, kRelocalizeWeakMaxVarTheta);
    if (measurement.covariance.m[0][0] > maxVarXY || measurement.covariance.m[1][1] > maxVarXY ||
        measurement.covariance.m[2][2] > maxVarTheta) {
        return false;
    }

    return measurement.confidence >= kRelocalizeWeakConfidence;
}

bool relocalizationRobustOutlierAccepted(const lemlib::localization::MCLMeasurement& measurement,
                                         const PoseResidualSummary& residuals,
                                         const lemlib::localization::LocalizationConfig& config) {
    if (!measurement.valid || measurement.activeSensors < kRelocalizeMinSensors) return false;
    if (residuals.scoredSensors < kRelocalizeCommitMinSensors || residuals.inlierSensors < kRelocalizeMinSensors) {
        return false;
    }
    if (measurement.confidence < config.fusion.minConfidence) return false;

    const float maxVarXY = std::min(config.fusion.maxVarXY, kRelocalizeWeakMaxVarXY);
    const float maxVarTheta = std::min(config.fusion.maxVarTheta, kRelocalizeWeakMaxVarTheta);
    if (measurement.covariance.m[0][0] > maxVarXY || measurement.covariance.m[1][1] > maxVarXY ||
        measurement.covariance.m[2][2] > maxVarTheta) {
        return false;
    }

    return residuals.meanAbsResidual <= kRelocalizeMaxMeanResidual &&
           residuals.maxAbsResidual <= kRelocalizeMaxResidual;
}

bool relocalizationWallDirectAccepted(const RelocalizationSummary& summary,
                                      const DirectWallSolveCandidate& candidate,
                                      const lemlib::localization::LocalizationConfig& config) {
    const bool varianceOk = summary.varX <= config.fusion.maxVarXY &&
                            summary.varY <= config.fusion.maxVarXY &&
                            summary.varTheta <= config.fusion.maxVarTheta;
    if (!varianceOk || summary.confidence < kRelocalizeMinConfidence) return false;

    if (summary.activeSensors >= kDirectWallCommitMinSensors &&
        summary.scoredSensors >= kDirectWallCommitMinSensors) {
        return true;
    }

    // On a square field, two sensors are often the only stable startup view.
    // Accept that only when the direct wall solve is geometrically complete:
    // at least one X-facing and one Y-facing sensor, both inliers, with much
    // tighter residuals than the normal 3+ sensor commit path.
    return candidate.xSensors >= 1 && candidate.ySensors >= 1 &&
           summary.activeSensors >= kDirectWallTwoAxisMinSensors &&
           summary.scoredSensors >= kDirectWallTwoAxisMinSensors &&
           summary.meanResidual <= kDirectWallTwoAxisMaxMeanResidual &&
           summary.maxResidual <= kDirectWallTwoAxisMaxResidual;
}

const char* relocalizationStatus(const lemlib::localization::MCLMeasurement& measurement,
                                 const lemlib::localization::LocalizationConfig& config) {
    if (!measurement.valid) return "no_valid_pose";
    if (measurement.activeSensors < kRelocalizeMinSensors) return "not_enough_live_sensors";
    if (measurement.activeSensors < kRelocalizeCommitMinSensors) return "not_enough_commit_sensors";
    if (relocalizationWeakAccepted(measurement, config)) return "weak_accept";
    if (measurement.confidence < kRelocalizeMinConfidence) return "low_confidence";
    if (measurement.covariance.m[0][0] > config.fusion.maxVarXY ||
        measurement.covariance.m[1][1] > config.fusion.maxVarXY ||
        measurement.covariance.m[2][2] > config.fusion.maxVarTheta) {
        return "high_variance";
    }
    return "accepted";
}

bool relocalizationMeasurementBetter(const lemlib::localization::MCLMeasurement& lhs,
                                     const lemlib::localization::MCLMeasurement& rhs,
                                     const lemlib::localization::LocalizationConfig& config) {
    const bool lhsAccepted = relocalizationAccepted(lhs, config);
    const bool rhsAccepted = relocalizationAccepted(rhs, config);
    if (lhsAccepted != rhsAccepted) return lhsAccepted;

    const bool lhsWeakAccepted = relocalizationWeakAccepted(lhs, config);
    const bool rhsWeakAccepted = relocalizationWeakAccepted(rhs, config);
    if (lhsWeakAccepted != rhsWeakAccepted) return lhsWeakAccepted;

    if (lhs.valid != rhs.valid) return lhs.valid;
    if (lhs.activeSensors != rhs.activeSensors) return lhs.activeSensors > rhs.activeSensors;
    if (std::fabs(lhs.confidence - rhs.confidence) > 1e-5f) return lhs.confidence > rhs.confidence;

    const float lhsVarXY = lhs.covariance.m[0][0] + lhs.covariance.m[1][1];
    const float rhsVarXY = rhs.covariance.m[0][0] + rhs.covariance.m[1][1];
    if (std::fabs(lhsVarXY - rhsVarXY) > 1e-5f) return lhsVarXY < rhsVarXY;
    if (std::fabs(lhs.covariance.m[2][2] - rhs.covariance.m[2][2]) > 1e-6f) {
        return lhs.covariance.m[2][2] < rhs.covariance.m[2][2];
    }

    return false;
}

bool relocalizationCandidateBetter(const RelocalizationCandidate& lhs, const RelocalizationCandidate& rhs,
                                   const lemlib::localization::LocalizationConfig& config) {
    if (lhs.hasMeasurement != rhs.hasMeasurement) return lhs.hasMeasurement;
    if (!lhs.hasMeasurement) return false;
    return relocalizationMeasurementBetter(lhs.measurement, rhs.measurement, config);
}

float toRadians(float theta, bool radians) {
    return radians ? theta : lemlib::degToRad(theta);
}

float fromRadians(float theta, bool radians) {
    const float wrapped = wrapRadians(theta);
    return radians ? wrapped : lemlib::radToDeg(wrapped);
}

lemlib::Pose localToAbsolutePose(const lemlib::Pose& originAbsRad, const lemlib::Pose& localPoseRad) {
    const float cosHeading = std::cos(originAbsRad.theta);
    const float sinHeading = std::sin(originAbsRad.theta);

    return lemlib::Pose(
        originAbsRad.x + localPoseRad.x * cosHeading + localPoseRad.y * sinHeading,
        originAbsRad.y - localPoseRad.x * sinHeading + localPoseRad.y * cosHeading,
        wrapRadians(originAbsRad.theta + localPoseRad.theta));
}

lemlib::Pose absoluteToLocalPose(const lemlib::Pose& originAbsRad, const lemlib::Pose& absolutePoseRad) {
    const float cosHeading = std::cos(originAbsRad.theta);
    const float sinHeading = std::sin(originAbsRad.theta);
    const float dx = absolutePoseRad.x - originAbsRad.x;
    const float dy = absolutePoseRad.y - originAbsRad.y;

    return lemlib::Pose(
        dx * cosHeading - dy * sinHeading,
        dx * sinHeading + dy * cosHeading,
        wrapRadians(absolutePoseRad.theta - originAbsRad.theta));
}

RelocalizationCandidate evaluateRelocalizationHeading(const lemlib::localization::LocalizationConfig& config,
                                                      const lemlib::localization::MCLConfig& baseConfig,
                                                      const std::array<lemlib::localization::SensorObservation, 4>& observations,
                                                      float heading, float headingStd, std::uint32_t deadlineMs) {
    RelocalizationCandidate candidate {};
    candidate.seedHeading = wrapRadians(heading);

    auto candidateConfig = baseConfig;
    candidateConfig.initStdTheta = headingStd;

    lemlib::localization::MCL relocalizer;
    relocalizer.configure(config.field, candidateConfig, config.sensors);
    relocalizer.reset(lemlib::Pose(0.0f, 0.0f, candidate.seedHeading));

    for (int i = 0; i < kRelocalizeIterationsPerHypothesis && pros::millis() < deadlineMs; ++i) {
        const auto measurement = relocalizer.update(observations);
        ++candidate.iterations;

        if (!candidate.hasMeasurement || relocalizationMeasurementBetter(measurement, candidate.measurement, config)) {
            candidate.measurement = measurement;
            candidate.hasMeasurement = true;
        }

        if (relocalizationAccepted(measurement, config)) {
            candidate.measurement = measurement;
            candidate.hasMeasurement = true;
            break;
        }
    }

    return candidate;
}

RelocalizationCandidate evaluateRelocalizationPoseSeed(const lemlib::localization::LocalizationConfig& config,
                                                       const lemlib::localization::MCLConfig& baseConfig,
                                                       const std::array<lemlib::localization::SensorObservation, 4>& observations,
                                                       const lemlib::Pose& seedPose, std::uint32_t deadlineMs) {
    RelocalizationCandidate candidate {};
    candidate.seedHeading = wrapRadians(seedPose.theta);

    lemlib::localization::MCL relocalizer;
    relocalizer.configure(config.field, baseConfig, config.sensors);
    relocalizer.reset(seedPose);

    for (int i = 0; i < kRelocalizeIterationsPerHypothesis && pros::millis() < deadlineMs; ++i) {
        const auto measurement = relocalizer.update(observations);
        ++candidate.iterations;

        if (!candidate.hasMeasurement || relocalizationMeasurementBetter(measurement, candidate.measurement, config)) {
            candidate.measurement = measurement;
            candidate.hasMeasurement = true;
        }

        if (relocalizationAccepted(measurement, config)) {
            candidate.measurement = measurement;
            candidate.hasMeasurement = true;
            break;
        }
    }

    return candidate;
}
} // namespace

void StartRelativeChassis::setOrigin(const lemlib::Pose& absolutePose, bool radians) {
    m_originAbsRad = absolutePose;
    if (!radians) m_originAbsRad.theta = lemlib::degToRad(m_originAbsRad.theta);
    m_originAbsRad.theta = wrapRadians(m_originAbsRad.theta);
    m_hasOrigin = true;
}

lemlib::Pose StartRelativeChassis::getOrigin(bool radians) const {
    lemlib::Pose pose = m_originAbsRad;
    pose.theta = fromRadians(pose.theta, radians);
    return pose;
}

void StartRelativeChassis::setPose(float x, float y, float theta, bool radians) {
    setPose(lemlib::Pose(x, y, theta), radians);
}

void StartRelativeChassis::setPose(lemlib::Pose pose, bool radians) {
    ensureLocalizationRunning();
    pose.theta = toRadians(pose.theta, radians);
    ::chassis.setPose(localToAbsolutePose(m_originAbsRad, pose), true);
}

lemlib::Pose StartRelativeChassis::getPose(bool radians, bool standardPos) const {
    // Resolve the local pose in the compass frame first: the origin heading is
    // stored as a compass angle, so absoluteToLocalPose must subtract it from a
    // compass absolute heading. Apply the standardPos convention to the LOCAL
    // heading afterwards -- mixing a standard-frame absolute with a compass
    // origin would leave the result wrong by 2*origin.theta.
    lemlib::Pose pose = absoluteToLocalPose(m_originAbsRad, ::chassis.getPose(true, false));
    if (standardPos) pose.theta = wrapRadians(M_PI_2 - pose.theta);
    pose.theta = fromRadians(pose.theta, radians);
    return pose;
}

void StartRelativeChassis::turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params, bool async) {
    const lemlib::Pose absoluteTarget = localToAbsolutePose(m_originAbsRad, lemlib::Pose(x, y, 0.0f));
    ::chassis.turnToPoint(absoluteTarget.x, absoluteTarget.y, timeout, params, async);
}

void StartRelativeChassis::turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params, bool async) {
    const float absoluteThetaDeg = lemlib::radToDeg(wrapRadians(m_originAbsRad.theta + lemlib::degToRad(theta)));
    ::chassis.turnToHeading(absoluteThetaDeg, timeout, params, async);
}

void StartRelativeChassis::swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
                                          lemlib::SwingToHeadingParams params, bool async) {
    const float absoluteThetaDeg = lemlib::radToDeg(wrapRadians(m_originAbsRad.theta + lemlib::degToRad(theta)));
    ::chassis.swingToHeading(absoluteThetaDeg, lockedSide, timeout, params, async);
}

void StartRelativeChassis::swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
                                        lemlib::SwingToPointParams params, bool async) {
    const lemlib::Pose absoluteTarget = localToAbsolutePose(m_originAbsRad, lemlib::Pose(x, y, 0.0f));
    ::chassis.swingToPoint(absoluteTarget.x, absoluteTarget.y, lockedSide, timeout, params, async);
}

void StartRelativeChassis::moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params,
                                      bool async) {
    const lemlib::Pose absoluteTarget =
        localToAbsolutePose(m_originAbsRad, lemlib::Pose(x, y, lemlib::degToRad(theta)));
    ::chassis.moveToPose(absoluteTarget.x, absoluteTarget.y, lemlib::radToDeg(absoluteTarget.theta), timeout, params,
                         async);
}

void StartRelativeChassis::moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params, bool async) {
    const lemlib::Pose absoluteTarget = localToAbsolutePose(m_originAbsRad, lemlib::Pose(x, y, 0.0f));
    ::chassis.moveToPoint(absoluteTarget.x, absoluteTarget.y, timeout, params, async);
}

void StartRelativeChassis::waitUntil(float dist) { ::chassis.waitUntil(dist); }

void StartRelativeChassis::waitUntilDone() { ::chassis.waitUntilDone(); }

void StartRelativeChassis::cancelMotion() { ::chassis.cancelMotion(); }

void StartRelativeChassis::cancelAllMotions() { ::chassis.cancelAllMotions(); }

void ensureLocalizationRunning() {
    if (!lemlib::localization::isRunning()) lemlib::localization::start();
}

void applyRelocalizedPose(const lemlib::Pose& pose) {
    ensureLocalizationRunning();
    chassis.setPose(pose, true);
}

RelocalizationSummary performGlobalRelocalization() {
    RelocalizationSummary summary {};
    summary.attempted = true;
    summary.distances = captureStableDistances();
    summary.headingMode = "heading_search";
    summary.pose = chassis.getPose(true);
    summary.imuHeading = currentHeadingRadians();
    summary.pose.theta = summary.imuHeading;
    const lemlib::Pose priorPose = summary.pose;
    const auto observations = makeSensorObservations(summary.distances);

    const auto config = lemlib::localization::getConfig();
    auto relocalizeLocConfig = config;
    for (auto& sensor : relocalizeLocConfig.sensors) {
        sensor.maxRange = std::max(sensor.maxRange, kRelocalizeStartupMaxRange);
        sensor.minConfidence = std::min(sensor.minConfidence, kRelocalizeStartupMinConfidence);
    }

    const int usableSnapshots = countUsableDistanceSnapshots(relocalizeLocConfig, summary.distances);
    if (usableSnapshots < kRelocalizeMinSensors) {
        summary.status = "not_enough_live_sensors";
        return summary;
    }

    DirectWallSolveCandidate bestWallSolve {};
    std::array<DirectWallSolveCandidate, 4> wallCandidates {};
    int wallCandidateCount = 0;
    const std::array<float, 4> axisHeadings {0.0f, kPiOver2, kPi, -kPiOver2};
    float bestWallScore = 1.0e9f;
    for (const float axisHeading : axisHeadings) {
        const auto candidate = solveDirectWallPose(relocalizeLocConfig, summary.distances, axisHeading, priorPose);
        if (!candidate.valid) continue;
        if (wallCandidateCount < static_cast<int>(wallCandidates.size())) wallCandidates[wallCandidateCount++] = candidate;
        // Bias selection toward the IMU heading so a 90/180 deg-rotated alias on
        // the near-square field cannot win on a marginally lower residual.
        const float headingPenalty =
            kRelocalizeHeadingPriorWeightPerRad * std::fabs(wrapRadians(candidate.pose.theta - summary.imuHeading));
        const float adjustedScore = candidate.selectionScore + headingPenalty;
        if (!bestWallSolve.valid || adjustedScore < bestWallScore) {
            bestWallSolve = candidate;
            bestWallScore = adjustedScore;
        }
    }

    if (bestWallSolve.valid) {
        summary.headingMode = "wall_direct";
        summary.pose = bestWallSolve.pose;
        const PoseResidualSummary residuals = scorePoseResiduals(relocalizeLocConfig, summary.distances, summary.pose);
        const float residualConfidence = std::clamp(1.0f - residuals.meanAbsResidual / 24.0f, 0.2f, 0.95f);
        const float maxResidualConfidence = std::clamp(
            1.0f - std::max(0.0f, residuals.maxAbsResidual - relocalizeLocConfig.mcl.outlierThreshold) / 24.0f,
            0.2f, 0.95f);
        summary.confidence = std::min(residualConfidence, maxResidualConfidence);
        const float varianceResidual = std::max(residuals.meanAbsResidual, residuals.maxAbsResidual * 0.5f);
        summary.varX = std::max(varianceResidual * varianceResidual, 4.0f);
        summary.varY = summary.varX;
        // Honest heading variance: the committed theta is a snapped cardinal, so
        // its uncertainty is at least its disagreement with the trusted IMU.
        const float wallHeadingDisagreement = std::fabs(wrapRadians(summary.pose.theta - summary.imuHeading));
        summary.varTheta = std::max(lemlib::degToRad(3.0f) * lemlib::degToRad(3.0f),
                                    wallHeadingDisagreement * wallHeadingDisagreement);
        summary.meanResidual = residuals.meanAbsResidual;
        summary.maxResidual = residuals.maxAbsResidual;
        summary.activeSensors = residuals.inlierSensors;
        summary.scoredSensors = residuals.scoredSensors;
        const bool normalResidualsAccepted = residualsAccepted(residuals, kDirectWallCommitMinSensors);
        const bool twoAxisResidualsAccepted =
            residualsAccepted(residuals, kDirectWallTwoAxisMinSensors, kDirectWallTwoAxisMaxMeanResidual,
                              kDirectWallTwoAxisMaxResidual);
        const bool wallHeadingConsistent = wallHeadingDisagreement <= kRelocalizeMaxHeadingDisagreementRad;
        if (wallHeadingConsistent && relocalizationWallDirectAccepted(summary, bestWallSolve, relocalizeLocConfig) &&
            (normalResidualsAccepted || twoAxisResidualsAccepted)) {
            summary.success = true;
            summary.status = "wall_direct";
            return summary;
        }

        if (usableSnapshots < kRelocalizeCommitMinSensors) {
            summary.status = !wallHeadingConsistent
                                 ? "imu_heading_mismatch"
                                 : (twoAxisResidualsAccepted ? "not_enough_commit_sensors" : "bad_direct_wall_residual");
            return summary;
        }
    }

    if (usableSnapshots < kRelocalizeCommitMinSensors) {
        summary.status = "not_enough_commit_sensors";
        return summary;
    }

    auto relocalizeWallConfig = relocalizeLocConfig;

    // Cross-hypothesis selection that layers the IMU heading prior on top of the
    // residual/confidence ordering: among two plausible candidates, prefer the
    // one whose heading agrees with the IMU when they disagree with it by
    // meaningfully different amounts. Breaks square-field 90/180 deg aliasing
    // without overriding a clearly-better (residual-accepted) solve.
    const float imuHeadingForPrior = summary.imuHeading;
    auto candidateBetterWithHeadingPrior = [&](const RelocalizationCandidate& lhs,
                                               const RelocalizationCandidate& rhs) -> bool {
        const bool lhsOk = lhs.hasMeasurement && relocalizationWeakAccepted(lhs.measurement, relocalizeWallConfig);
        const bool rhsOk = rhs.hasMeasurement && relocalizationWeakAccepted(rhs.measurement, relocalizeWallConfig);
        if (lhsOk && rhsOk) {
            const float lhsDis = std::fabs(wrapRadians(lhs.measurement.pose.theta - imuHeadingForPrior));
            const float rhsDis = std::fabs(wrapRadians(rhs.measurement.pose.theta - imuHeadingForPrior));
            if (std::fabs(lhsDis - rhsDis) > kRelocalizeHeadingTiebreakRad) return lhsDis < rhsDis;
        }
        return relocalizationCandidateBetter(lhs, rhs, relocalizeWallConfig);
    };

    auto relocalizeConfig = relocalizeLocConfig.mcl;
    relocalizeConfig.minActiveSensors = kRelocalizeMinSensors;
    relocalizeConfig.numParticles = std::max(relocalizeLocConfig.mcl.numParticles * 8, 2400);
    relocalizeConfig.sensorStd = std::min(relocalizeLocConfig.mcl.sensorStd, 2.0f);
    relocalizeConfig.outlierThreshold = std::min(relocalizeLocConfig.mcl.outlierThreshold, 5.0f);
    relocalizeConfig.outlierWeight = std::min(relocalizeLocConfig.mcl.outlierWeight, 0.12f);
    relocalizeConfig.initStdXY =
        std::max((relocalizeLocConfig.field.maxX - relocalizeLocConfig.field.minX) * 0.45f, 36.0f);
    relocalizeConfig.rougheningStdXY = std::max(relocalizeLocConfig.mcl.rougheningStdXY, 0.35f);
    relocalizeConfig.rougheningStdTheta = std::max(relocalizeLocConfig.mcl.rougheningStdTheta, 0.001f);
    relocalizeConfig.sideRougheningStdPerRad =
        std::max(relocalizeLocConfig.mcl.sideRougheningStdPerRad, 0.15f);
    relocalizeConfig.confidenceMaxSpread = std::max(relocalizeLocConfig.mcl.confidenceMaxSpread, 40.0f);
    summary.headingMode = wallCandidateCount > 0 ? "wall_seed_search" : "heading_search_walls";

    const std::uint32_t deadline = pros::millis() + kRelocalizeTimeoutMs;
    const float coarseStep = (2.0f * kPi) / static_cast<float>(kRelocalizeCoarseHeadingHypotheses);
    const float coarseHeadingStd = std::max(coarseStep * 0.45f, lemlib::degToRad(4.0f));
    const float refineStep = coarseStep * 0.25f;
    const float refineHeadingStd = std::max(refineStep * 0.75f, lemlib::degToRad(1.5f));
    RelocalizationCandidate bestCandidate {};
    bool haveBestCandidate = false;

    auto wallSeedConfig = relocalizeConfig;
    wallSeedConfig.minActiveSensors = kRelocalizeMinSensors;
    wallSeedConfig.numParticles = std::max(relocalizeLocConfig.mcl.numParticles * 4, 1600);
    wallSeedConfig.initStdXY = 6.0f;
    wallSeedConfig.initStdTheta = lemlib::degToRad(5.0f);

    for (int i = 0; i < wallCandidateCount && pros::millis() < deadline; ++i) {
        const auto candidate = evaluateRelocalizationPoseSeed(relocalizeWallConfig, wallSeedConfig, observations,
                                                              wallCandidates[i].pose, deadline);
        summary.iterations += candidate.iterations;
        ++summary.headingHypotheses;
        if (!haveBestCandidate || candidateBetterWithHeadingPrior(candidate, bestCandidate)) {
            bestCandidate = candidate;
            haveBestCandidate = true;
        }
    }

    if (!haveBestCandidate || !bestCandidate.hasMeasurement ||
        !relocalizationWeakAccepted(bestCandidate.measurement, relocalizeWallConfig)) {
        for (int i = 0; i < kRelocalizeCoarseHeadingHypotheses && pros::millis() < deadline; ++i) {
            const float heading = wrapRadians(-kPi + i * coarseStep);
            const auto candidate = evaluateRelocalizationHeading(relocalizeWallConfig, relocalizeConfig, observations,
                                                                 heading, coarseHeadingStd, deadline);
            summary.iterations += candidate.iterations;
            ++summary.headingHypotheses;
            if (!haveBestCandidate || candidateBetterWithHeadingPrior(candidate, bestCandidate)) {
                bestCandidate = candidate;
                haveBestCandidate = true;
            }
        }
    }

    if (haveBestCandidate) {
        const int halfWindow = kRelocalizeRefineHeadingHypotheses / 2;
        for (int offset = -halfWindow; offset <= halfWindow && pros::millis() < deadline; ++offset) {
            const float heading = wrapRadians(bestCandidate.seedHeading + offset * refineStep);
            const auto candidate = evaluateRelocalizationHeading(relocalizeWallConfig, relocalizeConfig, observations,
                                                                 heading, refineHeadingStd, deadline);
            summary.iterations += candidate.iterations;
            ++summary.headingHypotheses;
            if (candidateBetterWithHeadingPrior(candidate, bestCandidate)) {
                bestCandidate = candidate;
            }
        }
    }

    if (!haveBestCandidate || !bestCandidate.hasMeasurement) {
        summary.status = "no_valid_pose";
        return summary;
    }

    summary.pose = bestCandidate.measurement.pose;
    summary.pose.theta = wrapRadians(bestCandidate.measurement.pose.theta);
    const PoseResidualSummary residuals = scorePoseResiduals(relocalizeWallConfig, summary.distances, summary.pose);
    summary.confidence = bestCandidate.measurement.confidence;
    summary.varX = bestCandidate.measurement.covariance.m[0][0];
    summary.varY = bestCandidate.measurement.covariance.m[1][1];
    summary.varTheta = bestCandidate.measurement.covariance.m[2][2];
    summary.meanResidual = residuals.meanAbsResidual;
    summary.maxResidual = residuals.maxAbsResidual;
    summary.activeSensors = residuals.inlierSensors;
    summary.scoredSensors = residuals.scoredSensors;
    const bool filterAccepted = relocalizationAccepted(bestCandidate.measurement, relocalizeWallConfig) ||
                                relocalizationWeakAccepted(bestCandidate.measurement, relocalizeWallConfig);
    const bool residualAccepted = residualsAccepted(residuals, kRelocalizeCommitMinSensors);
    const bool robustOutlierAccepted =
        relocalizationRobustOutlierAccepted(bestCandidate.measurement, residuals, relocalizeWallConfig);
    summary.success = (filterAccepted && residualAccepted) || robustOutlierAccepted;
    summary.status = robustOutlierAccepted
                         ? "robust_outlier_accept"
                         : (filterAccepted && !residualAccepted
                                ? "bad_residual"
                                : relocalizationStatus(bestCandidate.measurement, relocalizeWallConfig));
    return summary;
}

bool beginAutonomousWithRelocalization(RelocalizationSummary* summary) {
    ensureLocalizationRunning();
    const RelocalizationSummary result = performGlobalRelocalization();
    if (summary != nullptr) *summary = result;
    if (!result.success) return false;

    applyRelocalizedPose(result.pose);
    return true;
}

bool beginAutonomousWithFixedStart(const lemlib::Pose& absoluteStartPose, StartRelativeChassis* localChassis) {
    ensureLocalizationRunning();
    ::chassis.setPose(absoluteStartPose, true);
    if (localChassis != nullptr) localChassis->setOrigin(absoluteStartPose, true);
    return true;
}

bool beginAutonomousWithRelocalizationOrFixedStart(const lemlib::Pose& fixedStartAbsolute,
                                                   StartRelativeChassis* localChassis,
                                                   RelocalizationSummary* summary) {
    ensureLocalizationRunning();
    // Robot must be stationary here: performGlobalRelocalization median-combines
    // several distance snapshots. The wall_direct solve is near-instant when two
    // perpendicular walls are in view; a worst-case MCL seed search can take up
    // to kRelocalizeTimeoutMs before it falls back.
    const RelocalizationSummary result = performGlobalRelocalization();
    if (summary != nullptr) *summary = result;

    // Anchor the start-relative route frame. On a confident solve, anchor to the
    // TRUE field pose so local route commands map to real field coordinates and
    // localization actively corrects the run. On failure, anchor to the
    // configured fixed start: the robot assumes its current position is the local
    // origin and runs the identical start-relative route on odometry while normal
    // gated range fusion stays tied to that track. Setting the chassis pose
    // resets odom and the localization filters to the anchor; the local origin
    // maps the robot's current physical pose to local (0, 0, 0) in both cases,
    // so the route is physically the same whether or not relocalization succeeded.
    const lemlib::Pose anchor = result.success ? result.pose : fixedStartAbsolute;
    ::chassis.setPose(anchor, true);
    if (localChassis != nullptr) localChassis->setOrigin(anchor, true);
    return result.success;
}
} // namespace autonomous_localization
