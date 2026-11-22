#include "diesel_vision_backend/arm/motor_angle_mapping.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

using namespace diesel_vision_backend::arm;

// 以下参数均为合成测试数据，绝非本机实际标定或限位。
TEST(MotorAngleMapping, UnconfiguredRejectsBothDirections) {
    const MotorAngleMapping mapping;
    EXPECT_FALSE(mapping.is_configured());
    EXPECT_FALSE(mapping.to_joint_angles({}).has_value());
    EXPECT_FALSE(mapping.to_motor_angles({}).has_value());
}

TEST(MotorAngleMapping, DirectionRatioAndZeroAreExplicit) {
    MotorAngleCalibration calibration;
    calibration.motor_to_joint.diagonal() << 1, -0.5, 2;
    calibration.motor_zero_rad << .1, .2, .3;
    calibration.joint_reference_rad << 0, .4, -.2;
    const MotorAngleMapping mapping(calibration);
    const auto joint = mapping.to_joint_angles({.3, .6, .5});
    ASSERT_TRUE(joint);
    EXPECT_TRUE(mapping.is_configured());
    EXPECT_NEAR(joint->base_yaw, .2, 1e-12);
    EXPECT_NEAR(joint->upper_elevation, .2, 1e-12);
    EXPECT_NEAR(joint->forearm_elevation, .2, 1e-12);
    const auto motor = mapping.to_motor_angles(*joint);
    ASSERT_TRUE(motor);
    EXPECT_NEAR(motor->base_motor, .3, 1e-12);
    EXPECT_NEAR(motor->upper_motor, .6, 1e-12);
    EXPECT_NEAR(motor->forearm_motor, .5, 1e-12);
}

TEST(MotorAngleMapping, CoupledForearmIsAbsoluteAngleAndFeedsForwardModel) {
    MotorAngleCalibration calibration;
    // 示例：小臂绝对角=大臂驱动角+肘部相对驱动角，禁止默认假定实物就是此结构。
    calibration.motor_to_joint << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    const MotorAngleMapping mapping(calibration);
    const auto joint = mapping.to_joint_angles({0, .6, -.8});
    ASSERT_TRUE(joint);
    EXPECT_NEAR(joint->forearm_elevation, -.2, 1e-12);
    const auto motor = mapping.to_motor_angles(*joint);
    ASSERT_TRUE(motor);
    EXPECT_NEAR(motor->forearm_motor, -.8, 1e-12);
    const LevelArmModel model;
    const auto state = model.forward(*joint);
    const auto expected = model.forward({0, .6, -.2});
    EXPECT_LT((state.base_endpoint.matrix() - expected.base_endpoint.matrix()).norm(), 1e-12);
}

TEST(MotorAngleMapping, ContinuousAnglesAreNotWrapped) {
    MotorAngleCalibration calibration;
    calibration.motor_to_joint.setIdentity();
    const MotorAngleMapping mapping(calibration);
    const auto joint = mapping.to_joint_angles({20, -10, 15});
    ASSERT_TRUE(joint);
    EXPECT_DOUBLE_EQ(joint->base_yaw, 20);
    const auto motor = mapping.to_motor_angles(*joint);
    ASSERT_TRUE(motor);
    EXPECT_DOUBLE_EQ(motor->upper_motor, -10);
    EXPECT_DOUBLE_EQ(motor->forearm_motor, 15);
}

TEST(MotorAngleMapping, RejectsInvalidCalibration) {
    MotorAngleCalibration calibration;
    EXPECT_THROW(MotorAngleMapping{calibration}, std::invalid_argument);
    calibration.motor_to_joint.setIdentity();
    calibration.motor_to_joint(2, 2) = 1e-12;
    EXPECT_THROW(MotorAngleMapping{calibration}, std::invalid_argument);
    calibration.motor_to_joint.setIdentity();
    calibration.motor_zero_rad.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(MotorAngleMapping{calibration}, std::invalid_argument);
    calibration.motor_zero_rad.setZero();
    calibration.joint_reference_rad.y() = std::numeric_limits<double>::infinity();
    EXPECT_THROW(MotorAngleMapping{calibration}, std::invalid_argument);
}

TEST(MotorAngleMapping, RejectsInvalidInputsAndOverflow) {
    MotorAngleCalibration calibration;
    calibration.motor_to_joint = 2 * Eigen::Matrix3d::Identity();
    const MotorAngleMapping mapping(calibration);
    EXPECT_FALSE(mapping.to_joint_angles({0, std::numeric_limits<double>::quiet_NaN(), 0}));
    EXPECT_FALSE(mapping.to_motor_angles({0, 0, std::numeric_limits<double>::infinity()}));
    EXPECT_FALSE(mapping.to_joint_angles({std::numeric_limits<double>::max(), 0, 0}));
    calibration.motor_to_joint = .5 * Eigen::Matrix3d::Identity();
    const MotorAngleMapping inverse_overflow(calibration);
    EXPECT_FALSE(inverse_overflow.to_motor_angles({std::numeric_limits<double>::max(), 0, 0}));
}
