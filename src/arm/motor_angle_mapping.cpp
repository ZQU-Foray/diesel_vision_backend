#include "diesel_vision_backend/arm/motor_angle_mapping.hpp"

#include <Eigen/LU>
#include <stdexcept>

namespace diesel_vision_backend::arm {

std::optional<Eigen::Vector3d>
MotorAngleMapping::to_motor_velocity(const Eigen::Vector3d& velocity) const {
    if (!calibration_ || !velocity.allFinite())
        return std::nullopt;
    const Eigen::Vector3d result = joint_to_motor_ * velocity;
    if (!result.allFinite())
        return std::nullopt;
    return result;
}

std::optional<Eigen::Vector3d>
MotorAngleMapping::to_joint_velocity(const Eigen::Vector3d& motor_velocity) const {
    if (!calibration_ || !motor_velocity.allFinite())
        return std::nullopt;
    const Eigen::Vector3d velocity = calibration_->motor_to_joint * motor_velocity;
    if (!velocity.allFinite())
        return std::nullopt;
    return velocity;
}

MotorAngleMapping::MotorAngleMapping(const MotorAngleCalibration& calibration) {
    if (!calibration.motor_to_joint.allFinite() || !calibration.motor_zero_rad.allFinite() ||
        !calibration.joint_reference_rad.allFinite()) {
        throw std::invalid_argument("motor angle calibration must be finite");
    }
    Eigen::FullPivLU<Eigen::Matrix3d> decomposition(calibration.motor_to_joint);
    // 相对主元阈值：拒绝数值退化的逆映射，不是实物精度或安全门限。
    decomposition.setThreshold(1e-10);
    if (!decomposition.isInvertible()) {
        throw std::invalid_argument("motor angle mapping must be numerically invertible");
    }
    joint_to_motor_ = decomposition.inverse();
    if (!joint_to_motor_.allFinite()) {
        throw std::invalid_argument("inverse motor angle mapping overflow");
    }
    calibration_ = calibration;
}

bool MotorAngleMapping::is_configured() const {
    return calibration_.has_value();
}

std::optional<LevelArmAngles>
MotorAngleMapping::to_joint_angles(const MotorAngles& motor_angles) const {
    if (!calibration_) {
        return std::nullopt;
    }
    const Eigen::Vector3d motor_vector(motor_angles.base_motor, motor_angles.upper_motor,
                                       motor_angles.forearm_motor);
    if (!motor_vector.allFinite()) {
        return std::nullopt;
    }
    const Eigen::Vector3d joint_vector =
        calibration_->motor_to_joint * (motor_vector - calibration_->motor_zero_rad) +
        calibration_->joint_reference_rad;
    if (!joint_vector.allFinite()) {
        return std::nullopt;
    }
    return LevelArmAngles{joint_vector.x(), joint_vector.y(), joint_vector.z()};
}

std::optional<MotorAngles>
MotorAngleMapping::to_motor_angles(const LevelArmAngles& joint_angles) const {
    if (!calibration_) {
        return std::nullopt;
    }
    const Eigen::Vector3d joint_vector(joint_angles.base_yaw, joint_angles.upper_elevation,
                                       joint_angles.forearm_elevation);
    if (!joint_vector.allFinite()) {
        return std::nullopt;
    }
    const Eigen::Vector3d motor_vector =
        joint_to_motor_ * (joint_vector - calibration_->joint_reference_rad) +
        calibration_->motor_zero_rad;
    if (!motor_vector.allFinite()) {
        return std::nullopt;
    }
    return MotorAngles{motor_vector.x(), motor_vector.y(), motor_vector.z()};
}

} // namespace diesel_vision_backend::arm
