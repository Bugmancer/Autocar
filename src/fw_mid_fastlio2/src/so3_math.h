#pragma once

// Small Eigen-only replacement for the Sophus operations used by this package.
// Keeping this local makes the ROS1 package buildable without an external
// Sophus installation while preserving the original right/left Jacobian math.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace Sophus
{
class SO3d
{
public:
    SO3d() : rotation_(Eigen::Matrix3d::Identity()) {}
    explicit SO3d(const Eigen::Matrix3d &rotation) : rotation_(rotation) {}

    Eigen::Matrix3d matrix() const { return rotation_; }

    Eigen::Vector3d log() const
    {
        Eigen::AngleAxisd aa(rotation_);
        if (aa.angle() < 1e-12)
            return Eigen::Vector3d::Zero();
        return aa.axis() * aa.angle();
    }

    static Eigen::Matrix3d hat(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d m;
        m << 0.0, -v.z(), v.y(),
             v.z(), 0.0, -v.x(),
             -v.y(), v.x(), 0.0;
        return m;
    }

    static SO3d exp(const Eigen::Vector3d &phi)
    {
        const double theta = phi.norm();
        const Eigen::Matrix3d K = hat(phi);
        Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
        if (theta < 1e-10)
            return SO3d(result + K + 0.5 * K * K);
        result += (std::sin(theta) / theta) * K;
        result += ((1.0 - std::cos(theta)) / (theta * theta)) * K * K;
        return SO3d(result);
    }

    static Eigen::Matrix3d leftJacobian(const Eigen::Vector3d &phi)
    {
        const double theta = phi.norm();
        const Eigen::Matrix3d K = hat(phi);
        if (theta < 1e-8)
            return Eigen::Matrix3d::Identity() + 0.5 * K + (1.0 / 6.0) * K * K;
        return Eigen::Matrix3d::Identity() +
               ((1.0 - std::cos(theta)) / (theta * theta)) * K +
               ((theta - std::sin(theta)) / (theta * theta * theta)) * K * K;
    }

    static Eigen::Matrix3d leftJacobianInverse(const Eigen::Vector3d &phi)
    {
        const double theta = phi.norm();
        const Eigen::Matrix3d K = hat(phi);
        if (theta < 1e-8)
            return Eigen::Matrix3d::Identity() - 0.5 * K + (1.0 / 12.0) * K * K;
        const double half_theta = 0.5 * theta;
        const double cot_half = std::cos(half_theta) / std::sin(half_theta);
        const double coefficient = (1.0 - half_theta * cot_half) / (theta * theta);
        return Eigen::Matrix3d::Identity() - 0.5 * K + coefficient * K * K;
    }

private:
    Eigen::Matrix3d rotation_;
};
}  // namespace Sophus
