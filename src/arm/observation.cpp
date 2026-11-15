#include "diesel_vision_backend/arm/observation.hpp"

namespace diesel_vision_backend::arm {
namespace {
bool valid_pose(const Eigen::Isometry3d& p) {
    // 仅检查SE(3)数值合法性，1e-6不是标定精度声明。
    return p.matrix().allFinite() &&
           (p.linear().transpose() * p.linear() - Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
           std::abs(p.linear().determinant() - 1.0) <= 1e-6 &&
           (p.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() <= 1e-6;
}
} // namespace

std::optional<Eigen::Isometry3d> target_in_base(const CameraTarget& target,
                                                const CameraTransformSample& sample, double now,
                                                PairingPolicy policy) {
    if (!target.valid || !sample.valid || !valid_pose(target.camera_target) ||
        !valid_pose(sample.base_camera) || target.camera_target.translation().z() <= 0.0 ||
        !std::isfinite(now) || !std::isfinite(target.stamp_s) || !std::isfinite(sample.stamp_s) ||
        !std::isfinite(policy.max_age_s) || !std::isfinite(policy.max_difference_s) ||
        policy.max_age_s < 0 || policy.max_difference_s < 0 || target.stamp_s < 0 ||
        sample.stamp_s < 0 || now < target.stamp_s || now < sample.stamp_s ||
        now - target.stamp_s > policy.max_age_s || now - sample.stamp_s > policy.max_age_s ||
        std::abs(target.stamp_s - sample.stamp_s) > policy.max_difference_s) {
        return std::nullopt;
    }
    return sample.base_camera * target.camera_target;
}
} // namespace diesel_vision_backend::arm
