#pragma once

#include <cmath>

namespace lemlib::localization {

inline float wrapAngle(float angle) { return std::remainder(angle, 2.0f * static_cast<float>(M_PI)); }

struct Mat3 {
        float m[3][3] = {};

        static Mat3 identity(float diag = 1.0f) {
            Mat3 out {};
            out.m[0][0] = diag;
            out.m[1][1] = diag;
            out.m[2][2] = diag;
            return out;
        }
};

inline Mat3 add(const Mat3& a, const Mat3& b) {
    Mat3 out {};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) { out.m[r][c] = a.m[r][c] + b.m[r][c]; }
    }
    return out;
}

inline Mat3 sub(const Mat3& a, const Mat3& b) {
    Mat3 out {};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) { out.m[r][c] = a.m[r][c] - b.m[r][c]; }
    }
    return out;
}

inline Mat3 mul(const Mat3& a, const Mat3& b) {
    Mat3 out {};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m[r][c] = 0.0f;
            for (int k = 0; k < 3; ++k) { out.m[r][c] += a.m[r][k] * b.m[k][c]; }
        }
    }
    return out;
}

inline Mat3 transpose(const Mat3& a) {
    Mat3 out {};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) { out.m[r][c] = a.m[c][r]; }
    }
    return out;
}

inline bool invert(const Mat3& a, Mat3& out) {
    const float a00 = a.m[0][0], a01 = a.m[0][1], a02 = a.m[0][2];
    const float a10 = a.m[1][0], a11 = a.m[1][1], a12 = a.m[1][2];
    const float a20 = a.m[2][0], a21 = a.m[2][1], a22 = a.m[2][2];

    const float b01 = a22 * a11 - a12 * a21;
    const float b11 = -a22 * a10 + a12 * a20;
    const float b21 = a21 * a10 - a11 * a20;

    float det = a00 * b01 + a01 * b11 + a02 * b21;
    if (std::fabs(det) < 1e-6f) return false;
    const float inv_det = 1.0f / det;

    out.m[0][0] = b01 * inv_det;
    out.m[0][1] = (-a22 * a01 + a02 * a21) * inv_det;
    out.m[0][2] = (a12 * a01 - a02 * a11) * inv_det;
    out.m[1][0] = b11 * inv_det;
    out.m[1][1] = (a22 * a00 - a02 * a20) * inv_det;
    out.m[1][2] = (-a12 * a00 + a02 * a10) * inv_det;
    out.m[2][0] = b21 * inv_det;
    out.m[2][1] = (-a21 * a00 + a01 * a20) * inv_det;
    out.m[2][2] = (a11 * a00 - a01 * a10) * inv_det;
    return true;
}
} // namespace lemlib::localization
