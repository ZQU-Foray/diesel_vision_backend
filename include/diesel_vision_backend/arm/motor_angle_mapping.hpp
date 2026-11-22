#pragma once

#include "diesel_vision_backend/arm/level_arm.hpp"

#include <optional>

namespace diesel_vision_backend::arm {

/// 三个电机通道的连续角度，单位rad；不是几何关节角，不自动处理编码器跨圈。
struct MotorAngles {
    double base_motor = 0.0;
    double upper_motor = 0.0;
    double forearm_motor = 0.0;
};

/// 调用方注入的线性传动参数，无实物默认值。
/// 矩阵列顺序：底座/大臂驱动/小臂驱动电机；行顺序：yaw/大臂绝对仰角/小臂绝对仰角。
/// q = motor_to_joint * (m - motor_zero_rad) + joint_reference_rad。
/// 支持线性耦合，不保证适合非线性机构；数学有效不代表完成实物标定。
struct MotorAngleCalibration {
    Eigen::Matrix3d motor_to_joint = Eigen::Matrix3d::Zero();
    Eigen::Vector3d motor_zero_rad = Eigen::Vector3d::Zero();
    Eigen::Vector3d joint_reference_rad = Eigen::Vector3d::Zero();
};

/// 纯计算的双向映射；默认未配置，绝不静默使用单位映射驱动实物。
class MotorAngleMapping {
  public:
    MotorAngleMapping() = default;
    /// 非有限、奇异或数值上近奇异的参数抛invalid_argument，不做实物可信度判断。
    explicit MotorAngleMapping(const MotorAngleCalibration& calibration);

    bool is_configured() const;
    /// 未配置、输入非有限或计算溢出时返回空；输出可直接用于理想FK，不表示可安全执行。
    std::optional<LevelArmAngles> to_joint_angles(const MotorAngles& motor_angles) const;
    /// 逆向转换；返回值尚未检查限位、速度、时间与碰撞，不得直接作为运动授权。
    std::optional<MotorAngles> to_motor_angles(const LevelArmAngles& joint_angles) const;

    /// 连续角速度线性映射qdot=A*mdot，不混入零位；非有限/未配置/溢出返回空。
    std::optional<Eigen::Vector3d> to_joint_velocity(const Eigen::Vector3d& motor_velocity) const;
    /// 连续速度/加速度的逆向线性映射，无零位项；非有限/未配置/溢出返回空。
    std::optional<Eigen::Vector3d> to_motor_velocity(const Eigen::Vector3d& joint_velocity) const;

  private:
    std::optional<MotorAngleCalibration> calibration_;
    Eigen::Matrix3d joint_to_motor_ = Eigen::Matrix3d::Zero();
};

} // namespace diesel_vision_backend::arm
