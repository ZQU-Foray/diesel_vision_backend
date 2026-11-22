#include "diesel_vision_backend/arm/target_angle_check.hpp"

#include <gtest/gtest.h>
#include <limits>

using namespace diesel_vision_backend::arm;

namespace {
// 全部为合成测试参数，不可复制为实物安全限位。
TargetAngleLimits test_limits() {
    return {-Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones(), -Eigen::Vector3d::Ones(),
            Eigen::Vector3d::Ones()};
}

MotorAngleMapping test_mapping(double ratio = 1) {
    MotorAngleCalibration calibration;
    calibration.motor_to_joint = ratio * Eigen::Matrix3d::Identity();
    return MotorAngleMapping(calibration);
}
} // namespace

TEST(TargetAngleCheck, MissingAndInvalidConfigurationReject) {
    EXPECT_EQ(check_target_angles({}, test_mapping(), std::nullopt).status,
              TargetAngleStatus::LimitsUnconfigured);
    auto limits = test_limits();
    limits.joint_min_rad.x() = limits.joint_max_rad.x();
    EXPECT_EQ(check_target_angles({}, test_mapping(), limits).status,
              TargetAngleStatus::InvalidLimits);
    limits = test_limits();
    limits.motor_max_rad.z() = std::numeric_limits<double>::infinity();
    EXPECT_EQ(check_target_angles({}, test_mapping(), limits).status,
              TargetAngleStatus::InvalidLimits);
    const auto missing_mapping = check_target_angles({}, MotorAngleMapping{}, test_limits());
    EXPECT_EQ(missing_mapping.status, TargetAngleStatus::MappingUnconfigured);
    EXPECT_FALSE(missing_mapping.motor_target);
}

TEST(TargetAngleCheck, InclusiveBoundsAndNoClamping) {
    const auto mapping = test_mapping();
    const auto limits = test_limits();
    const auto boundary = check_target_angles({-1, 1, 0}, mapping, limits);
    EXPECT_EQ(boundary.status, TargetAngleStatus::WithinStaticLimits);
    ASSERT_TRUE(boundary.motor_target);
    EXPECT_DOUBLE_EQ(boundary.motor_target->upper_motor, 1);
    const LevelArmAngles target{0, 1.01, 0};
    const auto outside = check_target_angles(target, mapping, limits);
    EXPECT_EQ(outside.status, TargetAngleStatus::JointLimitExceeded);
    EXPECT_EQ(outside.offending_axis, 1);
    EXPECT_FALSE(outside.motor_target);
    EXPECT_DOUBLE_EQ(target.upper_elevation, 1.01);
}

TEST(TargetAngleCheck, CouplingCanViolateMotorLimitsDespiteValidJointAngles) {
    MotorAngleCalibration calibration;
    calibration.motor_to_joint << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    const auto result =
        check_target_angles({0, -.8, .8}, MotorAngleMapping(calibration), test_limits());
    EXPECT_EQ(result.status, TargetAngleStatus::MotorLimitExceeded);
    EXPECT_EQ(result.offending_axis, 2);
    EXPECT_FALSE(result.motor_target);
}

TEST(TargetAngleCheck, EveryAxisAndBothSidesReject) {
    for (int axis = 0; axis < 3; ++axis) {
        for (double value : {-1.1, 1.1}) {
            Eigen::Vector3d values = Eigen::Vector3d::Zero();
            values[axis] = value;
            const LevelArmAngles target{values.x(), values.y(), values.z()};
            auto result = check_target_angles(target, test_mapping(), test_limits());
            EXPECT_EQ(result.status, TargetAngleStatus::JointLimitExceeded);
            EXPECT_EQ(result.offending_axis, axis);
            auto limits = test_limits();
            limits.joint_min_rad *= 2;
            limits.joint_max_rad *= 2;
            result = check_target_angles(target, test_mapping(), limits);
            EXPECT_EQ(result.status, TargetAngleStatus::MotorLimitExceeded);
            EXPECT_EQ(result.offending_axis, axis);
            EXPECT_FALSE(result.motor_target);
        }
    }
}

TEST(TargetAngleCheck, NonFiniteAndOverflowReject) {
    const auto invalid = check_target_angles({0, 0, std::numeric_limits<double>::quiet_NaN()},
                                             test_mapping(), test_limits());
    EXPECT_EQ(invalid.status, TargetAngleStatus::InvalidTarget);
    EXPECT_FALSE(invalid.motor_target);
    auto limits = test_limits();
    limits.joint_max_rad.x() = std::numeric_limits<double>::max();
    const auto overflow =
        check_target_angles({std::numeric_limits<double>::max(), 0, 0}, test_mapping(.5), limits);
    EXPECT_EQ(overflow.status, TargetAngleStatus::MappingFailed);
    EXPECT_FALSE(overflow.motor_target);
}
