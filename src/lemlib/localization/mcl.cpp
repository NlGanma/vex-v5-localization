#include "lemlib/localization/mcl.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace lemlib::localization {
namespace {
constexpr float kSqrt2Pi = 2.506628275f;
constexpr float kNoHit = 1e9f; // sentinel: no intersection found
constexpr float kNoHitThreshold = 1e8f; // any t above this is treated as no hit
constexpr float kMinVarXY = 0.01f; // variance floor for x/y (inches^2)
constexpr float kMinVarTheta = 0.0001f; // variance floor for theta (rad^2)
constexpr float kLargeT = 1e9f; // initial large-t for ray–AABB slab test
constexpr float kMotionNoiseReferenceDt = 0.05f; // interpret fixed motion-noise floors per 50 ms sensor cycle
constexpr float kMaxConfidence = 63.0f; // PROS distance sensor max confidence value
constexpr std::int32_t kConfidenceAvailableMinMm = 200; // PROS only reports confidence above 200 mm
constexpr float kMaxAccumulatedTurnForSideSpread = static_cast<float>(M_PI); // keep stale side spread bounded
constexpr float kGlobalInitFraction =
    0.25f; // initStdXY above this fraction of the field span is treated as global search

inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline float expNegApprox(float x) {
    if (x <= 1e-4f) return 1.0f;

    struct Segment {
            float upper;
            float center;
            float expCenter;
    };

    constexpr Segment kSegments[] = {
        {0.8f, 0.4f, 0.670320046f},
        {1.6f, 1.2f, 0.301194212f},
        {2.4f, 2.0f, 0.135335283f},
        {3.2f, 2.8f, 0.060810063f},
    };

    for (const auto& segment : kSegments) {
        if (x > segment.upper) continue;
        const float d = x - segment.center;
        const float poly = 1.0f - d + 0.5f * d * d - (d * d * d) / 6.0f;
        return std::max(0.0f, segment.expCenter * poly);
    }

    return std::exp(-x);
}

inline float gaussianPdf(float error, float sigma) {
    const float var = sigma * sigma;
    const float norm = 1.0f / (sigma * kSqrt2Pi);
    return norm * expNegApprox(0.5f * (error * error) / var);
}

inline void applyLocalOffset(float& x, float& y, float theta, float localX, float localY) {
    const float sinT = std::sin(theta);
    const float cosT = std::cos(theta);
    x += localY * sinT + localX * -cosT;
    y += localY * cosT + localX * sinT;
}

float raycastCircle(float sx, float sy, float dirX, float dirY, float cx, float cy, float radius) {
    const float dx = sx - cx;
    const float dy = sy - cy;
    const float a = dirX * dirX + dirY * dirY;
    const float b = 2.0f * (dx * dirX + dy * dirY);
    const float c = dx * dx + dy * dy - radius * radius;
    const float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) return -1.0f;
    const float sqrt_disc = std::sqrt(disc);
    const float inv2a = 1.0f / (2.0f * a);
    const float t1 = (-b - sqrt_disc) * inv2a;
    const float t2 = (-b + sqrt_disc) * inv2a;
    float t = kNoHit;
    if (t1 > 0.0f) t = std::min(t, t1);
    if (t2 > 0.0f) t = std::min(t, t2);
    if (t > kNoHitThreshold) return -1.0f;
    return t;
}

float raycastRect(float sx, float sy, float dirX, float dirY, float minX, float maxX, float minY, float maxY) {
    const float eps = 1e-6f;
    float tmin = -kLargeT;
    float tmax = kLargeT;

    if (std::fabs(dirX) < eps) {
        if (sx < minX || sx > maxX) return -1.0f;
    } else {
        const float tx1 = (minX - sx) / dirX;
        const float tx2 = (maxX - sx) / dirX;
        tmin = std::max(tmin, std::min(tx1, tx2));
        tmax = std::min(tmax, std::max(tx1, tx2));
    }

    if (std::fabs(dirY) < eps) {
        if (sy < minY || sy > maxY) return -1.0f;
    } else {
        const float ty1 = (minY - sy) / dirY;
        const float ty2 = (maxY - sy) / dirY;
        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));
    }

    if (tmax < 0.0f || tmin > tmax) return -1.0f;
    if (tmin >= 0.0f) return tmin;
    if (tmax >= 0.0f) return tmax;
    return -1.0f;
}
} // namespace

void MCL::configure(const FieldConfig& field, const MCLConfig& cfg, const std::array<SensorConfig, 4>& sensors) {
    field_ = field;
    cfg_ = cfg;
    cfg_.numParticles = std::max(cfg_.numParticles, 1);
    cfg_.sensorStd = std::max(cfg_.sensorStd, 1e-3f);
    cfg_.outlierThreshold = std::max(cfg_.outlierThreshold, 0.0f);
    cfg_.outlierWeight = clampf(cfg_.outlierWeight, 0.0f, 1.0f);
    cfg_.minWeight = std::max(cfg_.minWeight, 1e-12f);
    cfg_.resampleEssRatio = clampf(cfg_.resampleEssRatio, 0.0f, 1.0f);
    cfg_.motionStdXY = std::max(cfg_.motionStdXY, 0.0f);
    cfg_.motionStdXYPerIn = std::max(cfg_.motionStdXYPerIn, 0.0f);
    cfg_.motionStdTheta = std::max(cfg_.motionStdTheta, 0.0f);
    cfg_.motionStdThetaPerRad = std::max(cfg_.motionStdThetaPerRad, 0.0f);
    cfg_.motionLateralStdPerRad = std::max(cfg_.motionLateralStdPerRad, 0.0f);
    cfg_.motionLateralStdPerInRad = std::max(cfg_.motionLateralStdPerInRad, 0.0f);
    cfg_.motionLateralBiasPerRad = std::isfinite(cfg_.motionLateralBiasPerRad) ? cfg_.motionLateralBiasPerRad : 0.0f;
    cfg_.motionLateralBiasPerInRad =
        std::isfinite(cfg_.motionLateralBiasPerInRad) ? cfg_.motionLateralBiasPerInRad : 0.0f;
    const int configuredSensors = static_cast<int>(std::count_if(
        sensors.begin(), sensors.end(), [](const SensorConfig& sensor) { return sensor.sensor != nullptr; }));
    cfg_.minActiveSensors = std::clamp(cfg_.minActiveSensors, 1, std::max(configuredSensors, 1));
    cfg_.initStdXY = std::max(cfg_.initStdXY, 0.0f);
    cfg_.initStdTheta = std::max(cfg_.initStdTheta, 0.0f);
    cfg_.confidenceMaxSpread = std::max(cfg_.confidenceMaxSpread, 1e-3f);
    cfg_.rougheningStdXY = std::max(cfg_.rougheningStdXY, 0.0f);
    cfg_.rougheningStdTheta = std::max(cfg_.rougheningStdTheta, 0.0f);
    cfg_.sideRougheningStdPerRad = std::max(cfg_.sideRougheningStdPerRad, 0.0f);
    sensors_ = sensors;
    particles_.clear();
    particles_.reserve(static_cast<size_t>(cfg_.numParticles));
    accumulatedTurnMag_ = 0.0f;
    configured_ = true;
}

void MCL::reset(const lemlib::Pose& pose) {
    if (!configured_) return;

    particles_.clear();
    particles_.resize(static_cast<size_t>(cfg_.numParticles));
    accumulatedTurnMag_ = 0.0f;

    const float minX = std::min(field_.minX + field_.fieldMargin, field_.maxX - field_.fieldMargin);
    const float maxX = std::max(field_.minX + field_.fieldMargin, field_.maxX - field_.fieldMargin);
    const float minY = std::min(field_.minY + field_.fieldMargin, field_.maxY - field_.fieldMargin);
    const float maxY = std::max(field_.minY + field_.fieldMargin, field_.maxY - field_.fieldMargin);
    auto isInsideField = [&](float x, float y) { return x >= minX && x <= maxX && y >= minY && y <= maxY; };
    auto isValidParticlePose = [&](float x, float y) { return isInsideField(x, y) && !isInsideObstacle(x, y); };

    const bool useInitNoiseXY = cfg_.initStdXY > 0.0f;
    const bool useInitNoiseTheta = cfg_.initStdTheta > 0.0f;
    const float fieldSpan = std::min(maxX - minX, maxY - minY);
    const bool useGlobalUniformInitXY = fieldSpan > 0.0f && cfg_.initStdXY >= fieldSpan * kGlobalInitFraction;
    std::normal_distribution<float> dist_xy(0.0f, useInitNoiseXY ? cfg_.initStdXY : 1.0f);
    std::normal_distribution<float> dist_theta(0.0f, useInitNoiseTheta ? cfg_.initStdTheta : 1.0f);
    std::uniform_real_distribution<float> dist_field_x(minX, maxX);
    std::uniform_real_distribution<float> dist_field_y(minY, maxY);

    float fallbackX = clampf(pose.x, minX, maxX);
    float fallbackY = clampf(pose.y, minY, maxY);
    bool haveFallbackPose = isValidParticlePose(fallbackX, fallbackY);
    if (!haveFallbackPose) {
        for (int attempt = 0; attempt < 2048; ++attempt) {
            const float candidateX = dist_field_x(rng_);
            const float candidateY = dist_field_y(rng_);
            if (!isValidParticlePose(candidateX, candidateY)) continue;
            fallbackX = candidateX;
            fallbackY = candidateY;
            haveFallbackPose = true;
            break;
        }
    }
    if (!haveFallbackPose) {
        constexpr float kGridStep = 2.0f;
        for (float y = minY; y <= maxY && !haveFallbackPose; y += kGridStep) {
            for (float x = minX; x <= maxX; x += kGridStep) {
                if (!isValidParticlePose(x, y)) continue;
                fallbackX = x;
                fallbackY = y;
                haveFallbackPose = true;
                break;
            }
        }
    }

    for (auto& p : particles_) {
        bool placed = false;
        if (!useGlobalUniformInitXY) {
            for (int attempt = 0; attempt < 5; ++attempt) {
                p.x = pose.x + (useInitNoiseXY ? dist_xy(rng_) : 0.0f);
                p.y = pose.y + (useInitNoiseXY ? dist_xy(rng_) : 0.0f);
                if (isValidParticlePose(p.x, p.y)) {
                    placed = true;
                    break;
                }
            }
        }
        if (!placed) {
            if (!haveFallbackPose) {
                p.x = clampf(pose.x, minX, maxX);
                p.y = clampf(pose.y, minY, maxY);
            } else {
                for (int attempt = 0; attempt < 64; ++attempt) {
                    p.x = dist_field_x(rng_);
                    p.y = dist_field_y(rng_);
                    if (isValidParticlePose(p.x, p.y)) {
                        placed = true;
                        break;
                    }
                }
                if (!placed) {
                    p.x = fallbackX;
                    p.y = fallbackY;
                }
            }
        }
        p.theta = wrapAngle(pose.theta + (useInitNoiseTheta ? dist_theta(rng_) : 0.0f));
        p.weight = 1.0f / static_cast<float>(cfg_.numParticles);
    }
}

void MCL::predict(const lemlib::OdomDelta& delta) {
    if (!configured_ || particles_.empty()) return;

    const float dx = delta.localX;
    const float dy = delta.localY;
    const float dtheta = delta.deltaTheta;

    const float trans = std::sqrt(dx * dx + dy * dy);
    const float turnMag = std::fabs(dtheta);
    const float forwardMag = std::fabs(dy);
    // The fixed motion-noise floors are intended to model diffusion between
    // range updates, not to be charged at full strength on every tiny odom
    // tick. Scale them with sqrt(dt) so the same tuning remains reasonable
    // whether a move arrives as one large delta or many small deltas.
    const float dtSec = std::max(delta.dt, 0.0f);
    const float dtScale = (dtSec > 0.0f) ? std::sqrt(dtSec / kMotionNoiseReferenceDt) : 0.0f;
    const float sigma_xy = cfg_.motionStdXY * dtScale + cfg_.motionStdXYPerIn * trans;
    const float sigma_theta = cfg_.motionStdTheta * dtScale + cfg_.motionStdThetaPerRad * std::fabs(dtheta);
    const float sigma_lateral_turn =
        cfg_.motionLateralStdPerRad * turnMag + cfg_.motionLateralStdPerInRad * forwardMag * turnMag;
    const float lateralBias = dtheta * (cfg_.motionLateralBiasPerRad + cfg_.motionLateralBiasPerInRad * forwardMag);

    const bool useMotionNoiseXY = sigma_xy > 0.0f;
    const bool useMotionNoiseTheta = sigma_theta > 0.0f;
    const bool useMotionNoiseLateral = sigma_lateral_turn > 0.0f;
    std::normal_distribution<float> noise_xy(0.0f, useMotionNoiseXY ? sigma_xy : 1.0f);
    std::normal_distribution<float> noise_theta(0.0f, useMotionNoiseTheta ? sigma_theta : 1.0f);
    std::normal_distribution<float> noise_lateral_turn(0.0f, useMotionNoiseLateral ? sigma_lateral_turn : 1.0f);

    accumulatedTurnMag_ += turnMag;

    for (auto& p : particles_) {
        const float prev_x = p.x;
        const float prev_y = p.y;
        const float dx_n = dx + (useMotionNoiseXY ? noise_xy(rng_) : 0.0f) + lateralBias +
                           (useMotionNoiseLateral ? noise_lateral_turn(rng_) : 0.0f);
        const float dy_n = dy + (useMotionNoiseXY ? noise_xy(rng_) : 0.0f);
        const float dtheta_n = dtheta + (useMotionNoiseTheta ? noise_theta(rng_) : 0.0f);

        const float avgHeading = p.theta + dtheta_n * 0.5f;
        applyLocalOffset(p.x, p.y, avgHeading, dx_n, dy_n);
        p.theta = wrapAngle(p.theta + dtheta_n);

        p.x = clampf(p.x, field_.minX + field_.fieldMargin, field_.maxX - field_.fieldMargin);
        p.y = clampf(p.y, field_.minY + field_.fieldMargin, field_.maxY - field_.fieldMargin);

        if (isInsideObstacle(p.x, p.y)) {
            p.x = prev_x;
            p.y = prev_y;
        }
    }
}

MCLMeasurement MCL::update() {
    std::array<SensorObservation, 4> observations {};
    for (int i = 0; i < static_cast<int>(sensors_.size()); ++i) {
        const auto& s = sensors_[i];
        if (s.sensor == nullptr) continue;

        const std::int32_t dist_mm = s.sensor->get_distance();
        const bool validDistance = dist_mm > 0 && dist_mm < 9000;
        if (!validDistance) continue;

        observations[static_cast<size_t>(i)].available = true;
        observations[static_cast<size_t>(i)].distance = static_cast<float>(dist_mm) / 25.4f;
        if (dist_mm >= kConfidenceAvailableMinMm) {
            observations[static_cast<size_t>(i)].confidence = s.sensor->get_confidence();
        }
    }

    return update(observations);
}

MCLMeasurement MCL::update(const std::array<SensorObservation, 4>& observations) {
    MCLMeasurement out {};
    if (!configured_ || particles_.empty()) return out;

    struct ActiveSensor {
            int idx;
            float measurement;
            float confidenceScale;
    };

    std::array<ActiveSensor, 4> active {};
    int activeCount = 0;

    for (int i = 0; i < static_cast<int>(sensors_.size()); ++i) {
        const auto& s = sensors_[i];
        if (s.sensor == nullptr) continue;

        const auto& observation = observations[static_cast<size_t>(i)];
        const bool validDistance = observation.available && observation.distance > 0.0f;
        const bool confidenceAvailable = observation.confidence >= 0;
        const std::int32_t conf = observation.confidence;
        auto& sensorDebug = out.sensorReadings[static_cast<size_t>(i)];
        if (validDistance) sensorDebug.distance = observation.distance;
        if (confidenceAvailable) sensorDebug.confidence = conf;
        if (!validDistance) continue;

        const float dist_in = observation.distance;
        sensorDebug.inRange = dist_in >= s.minRange && dist_in <= s.maxRange;
        if (dist_in < s.minRange || dist_in > s.maxRange) continue;

        sensorDebug.confidenceAccepted =
            !confidenceAvailable ? true
                                 : (conf >= s.minConfidence && conf <= static_cast<std::int32_t>(kMaxConfidence));
        if (confidenceAvailable && !sensorDebug.confidenceAccepted) continue;

        const float confidenceScale =
            confidenceAvailable ? clampf(static_cast<float>(conf) / kMaxConfidence, 0.1f, 1.0f) : 1.0f;
        sensorDebug.used = true;
        active[activeCount++] = {i, dist_in, confidenceScale};
    }

    out.activeSensors = activeCount;
    if (activeCount < cfg_.minActiveSensors) {
        out.valid = false;
        return out;
    }

    const float sigma = std::max(cfg_.sensorStd, 1e-3f);
    const float uniformLikelihood = 1.0f / (sigma * kSqrt2Pi);
    float weights_sum = 0.0f;

    for (auto& p : particles_) {
        // Carry forward the previous normalized particle weight when resampling
        // is skipped so the posterior accumulates across scans.
        float w = std::isfinite(p.weight) ? p.weight : cfg_.minWeight;
        for (int k = 0; k < activeCount; ++k) {
            const auto& s = sensors_[active[k].idx];
            const float predicted = expectedDistance(p, s);
            const float confScale = active[k].confidenceScale;

            if (predicted <= 0.0f) {
                w *= cfg_.minWeight;
                continue;
            }
            const float error = predicted - active[k].measurement;
            if (std::fabs(error) > cfg_.outlierThreshold) {
                w *= cfg_.outlierWeight;
            } else {
                // Blend Gaussian likelihood toward uniform based on confidence
                const float likelihood = gaussianPdf(error, sigma);
                w *= confScale * likelihood + (1.0f - confScale) * uniformLikelihood;
            }
        }
        p.weight = w;
        weights_sum += w;
    }

    if (weights_sum < cfg_.minWeight) {
        const float uniform = 1.0f / static_cast<float>(cfg_.numParticles);
        for (auto& p : particles_) p.weight = uniform;
        weights_sum = 1.0f;
    } else {
        const float inv = 1.0f / weights_sum;
        for (auto& p : particles_) p.weight *= inv;
    }

    // Compute ESS before resample
    float ess = 0.0f;
    for (const auto& p : particles_) { ess += p.weight * p.weight; }
    ess = (ess > 0.0f) ? (1.0f / ess) : 0.0f;
    out.ess = ess;

    // Weighted mean (computed BEFORE resampling to use original importance weights)
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float mean_sin = 0.0f;
    float mean_cos = 0.0f;
    for (const auto& p : particles_) {
        const float w = p.weight;
        mean_x += p.x * w;
        mean_y += p.y * w;
        mean_sin += std::sin(p.theta) * w;
        mean_cos += std::cos(p.theta) * w;
    }
    const float mean_theta = std::atan2(mean_sin, mean_cos);

    // Covariance (diagonal, also before resampling)
    float var_x = 0.0f;
    float var_y = 0.0f;
    float var_theta = 0.0f;
    for (const auto& p : particles_) {
        const float dx = p.x - mean_x;
        const float dy = p.y - mean_y;
        const float dtheta = wrapAngle(p.theta - mean_theta);
        var_x += p.weight * dx * dx;
        var_y += p.weight * dy * dy;
        var_theta += p.weight * dtheta * dtheta;
    }

    // Resample if needed (systematic)
    if (ess < cfg_.resampleEssRatio * static_cast<float>(cfg_.numParticles)) {
        std::vector<Particle> resampled;
        resampled.resize(static_cast<size_t>(cfg_.numParticles));

        std::vector<float> cdf;
        cdf.resize(static_cast<size_t>(cfg_.numParticles));
        cdf[0] = particles_[0].weight;
        for (int i = 1; i < cfg_.numParticles; ++i) { cdf[i] = cdf[i - 1] + particles_[i].weight; }

        // Normalize CDF to guard against floating-point accumulation error
        const float cdfMax = cdf[cfg_.numParticles - 1];
        if (cdfMax > 0.0f) {
            const float invCdfMax = 1.0f / cdfMax;
            for (int i = 0; i < cfg_.numParticles; ++i) { cdf[i] *= invCdfMax; }
        }

        std::uniform_real_distribution<float> dist(0.0f, 1.0f / static_cast<float>(cfg_.numParticles));
        const float start = dist(rng_);
        for (int i = 0; i < cfg_.numParticles; ++i) {
            const float point = start + i * (1.0f / static_cast<float>(cfg_.numParticles));
            const auto it = std::lower_bound(cdf.begin(), cdf.end(), point);
            int idx = static_cast<int>(std::distance(cdf.begin(), it));
            if (idx >= cfg_.numParticles) idx = cfg_.numParticles - 1;
            resampled[i] = particles_[idx];
            resampled[i].weight = 1.0f / static_cast<float>(cfg_.numParticles);
        }
        particles_.swap(resampled);

        // Roughening: add small noise to resampled particles to prevent deprivation
        const float sideRougheningStd =
            cfg_.sideRougheningStdPerRad * std::min(accumulatedTurnMag_, kMaxAccumulatedTurnForSideSpread);
        if (cfg_.rougheningStdXY > 0.0f || cfg_.rougheningStdTheta > 0.0f || sideRougheningStd > 0.0f) {
            const bool useRoughXY = cfg_.rougheningStdXY > 0.0f;
            const bool useRoughTheta = cfg_.rougheningStdTheta > 0.0f;
            const bool useRoughSide = sideRougheningStd > 0.0f;
            std::normal_distribution<float> roughXY(0.0f, useRoughXY ? cfg_.rougheningStdXY : 1.0f);
            std::normal_distribution<float> roughTheta(0.0f, useRoughTheta ? cfg_.rougheningStdTheta : 1.0f);
            std::normal_distribution<float> roughSide(0.0f, useRoughSide ? sideRougheningStd : 1.0f);
            for (auto& p : particles_) {
                const float prevX = p.x;
                const float prevY = p.y;
                const float localSide = (useRoughXY ? roughXY(rng_) : 0.0f) + (useRoughSide ? roughSide(rng_) : 0.0f);
                const float localForward = useRoughXY ? roughXY(rng_) : 0.0f;
                applyLocalOffset(p.x, p.y, p.theta, localSide, localForward);
                p.theta = wrapAngle(p.theta + (useRoughTheta ? roughTheta(rng_) : 0.0f));
                p.x = clampf(p.x, field_.minX + field_.fieldMargin, field_.maxX - field_.fieldMargin);
                p.y = clampf(p.y, field_.minY + field_.fieldMargin, field_.maxY - field_.fieldMargin);
                if (isInsideObstacle(p.x, p.y)) {
                    p.x = prevX;
                    p.y = prevY;
                }
            }
        }
    }

    lemlib::Pose estimate {mean_x, mean_y, mean_theta};
    out.pose = estimate;
    out.covariance = Mat3::identity(1.0f);
    out.covariance.m[0][0] = std::max(var_x, kMinVarXY);
    out.covariance.m[1][1] = std::max(var_y, kMinVarXY);
    out.covariance.m[2][2] = std::max(var_theta, kMinVarTheta);

    const float spread = std::sqrt(var_x + var_y);
    const float ess_ratio = (cfg_.numParticles > 0) ? (ess / static_cast<float>(cfg_.numParticles)) : 0.0f;
    out.confidence = clampf(ess_ratio * (1.0f - (spread / cfg_.confidenceMaxSpread)), 0.0f, 1.0f);
    out.valid = true;
    accumulatedTurnMag_ = 0.0f;
    return out;
}

float MCL::expectedDistance(const Particle& p, const SensorConfig& sensor) const {
    // Sensor position in global frame
    const float sinT = std::sin(p.theta);
    const float cosT = std::cos(p.theta);

    const float sx = p.x + sensor.dx * cosT + sensor.dy * sinT;
    const float sy = p.y - sensor.dx * sinT + sensor.dy * cosT;

    const float heading = wrapAngle(p.theta + sensor.dtheta);
    const float dirX = std::sin(heading);
    const float dirY = std::cos(heading);

    return raycastToField(sx, sy, dirX, dirY);
}

float MCL::raycastToField(float sx, float sy, float dirX, float dirY) const {
    const float eps = 1e-6f;
    float t_min = kNoHit;

    if (std::fabs(dirX) > eps) {
        const float t1 = (field_.minX - sx) / dirX;
        const float y1 = sy + t1 * dirY;
        if (t1 > 0.0f && y1 >= field_.minY && y1 <= field_.maxY) t_min = std::min(t_min, t1);

        const float t2 = (field_.maxX - sx) / dirX;
        const float y2 = sy + t2 * dirY;
        if (t2 > 0.0f && y2 >= field_.minY && y2 <= field_.maxY) t_min = std::min(t_min, t2);
    }

    if (std::fabs(dirY) > eps) {
        const float t3 = (field_.minY - sy) / dirY;
        const float x3 = sx + t3 * dirX;
        if (t3 > 0.0f && x3 >= field_.minX && x3 <= field_.maxX) t_min = std::min(t_min, t3);

        const float t4 = (field_.maxY - sy) / dirY;
        const float x4 = sx + t4 * dirX;
        if (t4 > 0.0f && x4 >= field_.minX && x4 <= field_.maxX) t_min = std::min(t_min, t4);
    }
    // Keep obstacle geometry for particle validity, but do not use it to
    // occlude runtime distance-sensor rays. On-robot logs show the side/back
    // sensors consistently returning perimeter-wall distances where the map's
    // low-profile goal supports would otherwise block the ray, which broadens
    // and mis-centers the particle cloud during motion.

    if (t_min > kNoHitThreshold) return -1.0f;
    return t_min;
}

bool MCL::isInsideObstacle(float x, float y) const {
    const float margin = field_.obstacleMargin;
    for (const auto& obstacle : field_.obstacles) {
        if (obstacle.type == FieldConfig::Obstacle::Type::Circle) {
            const float dx = x - obstacle.x;
            const float dy = y - obstacle.y;
            const float r = obstacle.radius + margin;
            if (dx * dx + dy * dy <= r * r) return true;
        } else {
            if (std::fabs(x - obstacle.x) <= (obstacle.halfW + margin) &&
                std::fabs(y - obstacle.y) <= (obstacle.halfH + margin)) {
                return true;
            }
        }
    }
    return false;
}
} // namespace lemlib::localization
