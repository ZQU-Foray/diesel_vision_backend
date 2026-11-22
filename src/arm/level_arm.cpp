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

LevelArmState LevelArmModel::forward(const LevelArmAngles& angles) const {
    if (!std::isfinite(angles.base_yaw) || !std::isfinite(angles.upper_elevation) ||
        !std::isfinite(angles.forearm_elevation)) {
        throw std::invalid_argument("geometric angles must be finite");
    }
    // 两根杆分别向水平和竖直方向贡献多少距离，单位m。
    // 小臂仰角相对水平面；朝下时竖直贡献为负，不再叠加大臂角。
    const double upper_horizontal = geometry_.upper_length_m * std::cos(angles.upper_elevation);
    const double upper_vertical = geometry_.upper_length_m * std::sin(angles.upper_elevation);
    const double forearm_horizontal =
        geometry_.forearm_length_m * std::cos(angles.forearm_elevation);
    const double forearm_vertical = geometry_.forearm_length_m * std::sin(angles.forearm_elevation);
    const double endpoint_horizontal = upper_horizontal + forearm_horizontal;
    const double endpoint_height = upper_vertical + forearm_vertical;
    const double cos_base_yaw = std::cos(angles.base_yaw);
    const double sin_base_yaw = std::sin(angles.base_yaw);

    LevelArmState state;
    state.elbow_m = {upper_horizontal * cos_base_yaw, upper_horizontal * sin_base_yaw,
                     upper_vertical};
    state.base_endpoint = Eigen::Isometry3d::Identity();
    state.base_endpoint.translation() = Eigen::Vector3d(
        endpoint_horizontal * cos_base_yaw, endpoint_horizontal * sin_base_yaw, endpoint_height);
    // 理想平行连杆使末端不随大小臂俯仰，但仍跟随底座yaw。
    state.base_endpoint.linear() =
        Eigen::AngleAxisd(angles.base_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return state;
}

} // namespace diesel_vision_backend::arm
