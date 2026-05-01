#include "lemlib/localization/ekf.hpp"

#include <cmath>

namespace lemlib::localization {
namespace {
constexpr float kMinVarXY = 1e-6f;
constexpr float kMinVarTheta = 1e-8f;

void stabilizeCovariance(Mat3& covariance) {
    for (int row = 0; row < 3; ++row) {
        for (int col = row + 1; col < 3; ++col) {
            const float a = covariance.m[row][col];
            const float b = covariance.m[col][row];
            const float avg = (std::isfinite(a) && std::isfinite(b)) ? 0.5f * (a + b) : 0.0f;
            covariance.m[row][col] = avg;
            covariance.m[col][row] = avg;
        }
    }

    covariance.m[0][0] =
        (std::isfinite(covariance.m[0][0]) && covariance.m[0][0] > kMinVarXY) ? covariance.m[0][0] : kMinVarXY;
    covariance.m[1][1] =
        (std::isfinite(covariance.m[1][1]) && covariance.m[1][1] > kMinVarXY) ? covariance.m[1][1] : kMinVarXY;
    covariance.m[2][2] =
        (std::isfinite(covariance.m[2][2]) && covariance.m[2][2] > kMinVarTheta) ? covariance.m[2][2] : kMinVarTheta;
}
} // namespace

void EKF::predict(const lemlib::OdomDelta& delta) {
    const float dx = delta.localX;
    const float dy = delta.localY;
    const float dtheta = delta.deltaTheta;

    const float avgHeading = state_.theta + dtheta * 0.5f;
    const float sinA = std::sin(avgHeading);
    const float cosA = std::cos(avgHeading);

    // State prediction (matches LemLib odom integration)
    state_.x += dy * sinA + dx * -cosA;
    state_.y += dy * cosA + dx * sinA;
    state_.theta = wrapAngle(state_.theta + dtheta);

    // Jacobian wrt state
    Mat3 F = Mat3::identity(1.0f);
    F.m[0][2] = dy * cosA + dx * sinA;
    F.m[1][2] = -dy * sinA + dx * cosA;

    // Jacobian wrt control
    Mat3 G {};
    G.m[0][0] = -cosA; // d x / d dx
    G.m[0][1] = sinA; // d x / d dy
    G.m[0][2] = 0.5f * (dy * cosA + dx * sinA); // d x / d dtheta

    G.m[1][0] = sinA; // d y / d dx
    G.m[1][1] = cosA; // d y / d dy
    G.m[1][2] = 0.5f * (-dy * sinA + dx * cosA); // d y / d dtheta

    G.m[2][2] = 1.0f;

    const float trans = std::sqrt(dx * dx + dy * dy);
    const float turnMag = std::fabs(dtheta);
    const float forwardMag = std::fabs(dy);
    const float sigma_xy = std::max(0.0f, config_.processStdXY) + std::max(0.0f, config_.processStdXYPerIn) * trans;
    const float sigma_x = sigma_xy + std::max(0.0f, config_.processStdLateralPerRad) * turnMag +
                          std::max(0.0f, config_.processStdLateralPerInRad) * forwardMag * turnMag;
    const float sigma_y = sigma_xy;
    const float sigma_theta =
        std::max(0.0f, config_.processStdTheta) + std::max(0.0f, config_.processStdThetaPerRad) * turnMag;

    Mat3 Q {};
    Q.m[0][0] = sigma_x * sigma_x;
    Q.m[1][1] = sigma_y * sigma_y;
    Q.m[2][2] = sigma_theta * sigma_theta;

    // P = F P F^T + G Q G^T
    Mat3 FP = mul(F, P_);
    Mat3 FPFt = mul(FP, transpose(F));
    Mat3 GQ = mul(G, Q);
    Mat3 GQGt = mul(GQ, transpose(G));
    P_ = add(FPFt, GQGt);
    stabilizeCovariance(P_);
}

// Observation model: H = I (identity). MCL provides a full pose estimate,
// so the innovation is simply z - x and S = P + R.
void EKF::update(const lemlib::Pose& measurement, const Mat3& R) {
    // Innovation
    const float y0 = measurement.x - state_.x;
    const float y1 = measurement.y - state_.y;
    const float y2 = wrapAngle(measurement.theta - state_.theta);

    Mat3 S = add(P_, R);
    Mat3 S_inv {};
    if (!invert(S, S_inv)) return;

    // K = P * S^-1
    Mat3 K = mul(P_, S_inv);

    // State update: x = x + K * y
    state_.x += K.m[0][0] * y0 + K.m[0][1] * y1 + K.m[0][2] * y2;
    state_.y += K.m[1][0] * y0 + K.m[1][1] * y1 + K.m[1][2] * y2;
    state_.theta = wrapAngle(state_.theta + (K.m[2][0] * y0 + K.m[2][1] * y1 + K.m[2][2] * y2));

    // Joseph form: P = (I - K*H)*P*(I - K*H)' + K*R*K'  (with H = I)
    // More numerically stable than the simple P = (I - K)*P form.
    Mat3 I = Mat3::identity(1.0f);
    Mat3 IK = sub(I, K);
    Mat3 IKP = mul(IK, P_);
    Mat3 IKPIKt = mul(IKP, transpose(IK));
    Mat3 KR = mul(K, R);
    Mat3 KRKt = mul(KR, transpose(K));
    P_ = add(IKPIKt, KRKt);
    stabilizeCovariance(P_);
}

float EKF::innovationNIS(const lemlib::Pose& measurement, const Mat3& R) const {
    const float y0 = measurement.x - state_.x;
    const float y1 = measurement.y - state_.y;
    const float y2 = wrapAngle(measurement.theta - state_.theta);

    Mat3 S = add(P_, R);
    Mat3 S_inv {};
    if (!invert(S, S_inv)) return 1e9f;

    const float a0 = S_inv.m[0][0] * y0 + S_inv.m[0][1] * y1 + S_inv.m[0][2] * y2;
    const float a1 = S_inv.m[1][0] * y0 + S_inv.m[1][1] * y1 + S_inv.m[1][2] * y2;
    const float a2 = S_inv.m[2][0] * y0 + S_inv.m[2][1] * y1 + S_inv.m[2][2] * y2;
    return y0 * a0 + y1 * a1 + y2 * a2;
}
} // namespace lemlib::localization
