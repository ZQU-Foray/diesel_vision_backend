#include "diesel_vision_backend/arm/target_angle_check.hpp"

namespace diesel_vision_backend::arm {
namespace {

bool valid_bounds(const Eigen::Vector3d& minimum, const Eigen::Vector3d& maximum) {
    return minimum.allFinite() && maximum.allFinite() && (minimum.array() < maximum.array()).all();
}

int first_outside(const Eigen::Vector3d& angles, const Eigen::Vector3d& minimum,
                  const Eigen::Vector3d& maximum) {
    for (int axis = 0; axis < 3; ++axis) {
        if (angles[axis] < minimum[axis] || angles[axis] > maximum[axis]) {
            return axis;
        }
    }
    return -1;
}

} // namespace

TargetAngleCheck check_target_angles(const LevelArmAngles& target, const MotorAngleMapping& mapping,
                                     const std::optional<TargetAngleLimits>& limits) {
    if (!limits) {
        return {TargetAngleStatus::LimitsUnconfigured, -1, std::nullopt};
    }
    if (!valid_bounds(limits->joint_min_rad, limits->joint_max_rad) ||
        !valid_bounds(limits->motor_min_rad, limits->motor_max_rad)) {
        return {TargetAngleStatus::InvalidLimits, -1, std::nullopt};
    }
    const Eigen::Vector3d joint_target(target.base_yaw, target.upper_elevation,
                                       target.forearm_elevation);
    if (!joint_target.allFinite()) {
        return {TargetAngleStatus::InvalidTarget, -1, std::nullopt};
    }
    const int joint_axis =
        first_outside(joint_target, limits->joint_min_rad, limits->joint_max_rad);
    if (joint_axis >= 0) {
        return {TargetAngleStatus::JointLimitExceeded, joint_axis, std::nullopt};
    }
    if (!mapping.is_configured()) {
        return {TargetAngleStatus::MappingUnconfigured, -1, std::nullopt};
    }
    const auto motor_target = mapping.to_motor_angles(target);
    if (!motor_target) {
        return {TargetAngleStatus::MappingFailed, -1, std::nullopt};
    }
    const Eigen::Vector3d motor_vector(motor_target->base_motor, motor_target->upper_motor,
                                       motor_target->forearm_motor);
    const int motor_axis =
        first_outside(motor_vector, limits->motor_min_rad, limits->motor_max_rad);
    if (motor_axis >= 0) {
        return {TargetAngleStatus::MotorLimitExceeded, motor_axis, std::nullopt};
    }
    return {TargetAngleStatus::WithinStaticLimits, -1, motor_target};
}

} // namespace diesel_vision_backend::arm
