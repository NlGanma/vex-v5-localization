#pragma once

#include <cmath>
#include "lemlib/chassis/odom.hpp"
#include "lemlib/localization/math.hpp"
#include "lemlib/pose.hpp"

namespace lemlib::localization {

struct EKFConfig { // TUNE
        // Process noise (std dev)
        float processStdXY = 0.05f; // inches
        float processStdXYPerIn = 0.02f; // inches per inch traveled
        float processStdTheta = 0.01f; // radians
        float processStdThetaPerRad = 0.02f; // radians per radian rotated
        float processStdLateralPerRad = 0.0f; // inches of extra side uncertainty per radian turned
        float processStdLateralPerInRad = 0.0f; // inches of extra side uncertainty per inch-forward*radian-turn
};

class EKF {
    public:
        void setConfig(const EKFConfig& cfg) { config_ = cfg; }

        void reset(const lemlib::Pose& pose, const Mat3& covariance) {
            state_ = pose;
            P_ = covariance;
        }

        const lemlib::Pose& state() const { return state_; }

        const Mat3& covariance() const { return P_; }

        void predict(const lemlib::OdomDelta& delta);
        void update(const lemlib::Pose& measurement, const Mat3& R);

        float innovationNIS(const lemlib::Pose& measurement, const Mat3& R) const;
    private:
        EKFConfig config_ {};
        lemlib::Pose state_ {0, 0, 0};
        Mat3 P_ = Mat3::identity(1.0f);
};
} // namespace lemlib::localization
