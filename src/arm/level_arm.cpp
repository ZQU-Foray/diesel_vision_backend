#include "diesel_vision_backend/arm/level_arm.hpp"

#include <cmath>
#include <stdexcept>

namespace diesel_vision_backend::arm {

LevelArmModel::LevelArmModel(LevelArmGeometry geometry) : geometry_(geometry) {
    if (!std::isfinite(geometry.upper_length_m) || geometry.upper_length_m <= 0.0 ||
        !std::isfinite(geometry.forearm_length_m) || geometry.forearm_length_m <= 0.0) {
        throw std::invalid_argument("link lengths must be finite and positive");
    }
}

LevelArmState LevelArmModel::forward(const LevelArmAngles& a) const {
    if (!std::isfinite(a.base_yaw) || !std::isfinite(a.upper_elevation) ||
        !std::isfinite(a.forearm_elevation)) {
        throw std::invalid_argument("geometric angles must be finite");
    }
    const double upper_r = geometry_.upper_length_m * std::cos(a.upper_elevation);
    const double upper_z = geometry_.upper_length_m * std::sin(a.upper_elevation);
    const double radius = upper_r + geometry_.forearm_length_m * std::cos(a.forearm_elevation);
    const double height = upper_z + geometry_.forearm_length_m * std::sin(a.forearm_elevation);
    const double cy = std::cos(a.base_yaw), sy = std::sin(a.base_yaw);

    LevelArmState state;
    state.elbow_m = {upper_r * cy, upper_r * sy, upper_z};
    state.base_endpoint = Eigen::Isometry3d::Identity();
    state.base_endpoint.translation() = Eigen::Vector3d(radius * cy, radius * sy, height);
    // 理想平行连杆使末端不随大小臂俯仰，但仍跟随底座yaw。
    state.base_endpoint.linear() =
        Eigen::AngleAxisd(a.base_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return state;
}

} // namespace diesel_vision_backend::arm
