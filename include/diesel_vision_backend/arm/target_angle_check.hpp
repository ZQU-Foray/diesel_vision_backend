#pragma once

#include "diesel_vision_backend/arm/motor_angle_mapping.hpp"

#include <optional>

namespace diesel_vision_backend::arm {

/// 按yaw/大臂绝对角/小臂绝对角顺序配置几何域，电机域按MotorAngles顺序，单位rad。
/// 每轴须有有限且严格递增的上下界；默认零值配置无效，不代表实物限位。
struct TargetAngleLimits {
    Eigen::Vector3d joint_min_rad = Eigen::Vector3d::Zero();
    Eigen::Vector3d joint_max_rad = Eigen::Vector3d::Zero();
    Eigen::Vector3d motor_min_rad = Eigen::Vector3d::Zero();
    Eigen::Vector3d motor_max_rad = Eigen::Vector3d::Zero();
};

enum class TargetAngleStatus {
    WithinStaticLimits,
    LimitsUnconfigured,
    InvalidLimits,
    InvalidTarget,
    JointLimitExceeded,
    MappingUnconfigured,
    MappingFailed,
    MotorLimitExceeded
};

struct TargetAngleCheck {
    TargetAngleStatus status = TargetAngleStatus::LimitsUnconfigured;
    int offending_axis = -1; ///< 超限域的轴下标0..2，其余情况为-1；不是实体电机ID。
    std::optional<MotorAngles> motor_target; ///< 只有静态检查通过才有值，不等于允许发送。
};

/// 目标状态检查，不检查运动路径/当前状态/时效/近奇异，不夹紧或修改输入。
/// limits为空或非法时明确拒绝；映射与限位必须由同一套实际机构约定提供。
TargetAngleCheck check_target_angles(const LevelArmAngles& target, const MotorAngleMapping& mapping,
                                     const std::optional<TargetAngleLimits>& limits);

} // namespace diesel_vision_backend::arm
