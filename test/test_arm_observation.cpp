#include "diesel_vision_backend/arm/observation.hpp"
#include <gtest/gtest.h>

TEST(ArmObservation, TransformsAndRejectsMismatchedSamples) {
    using namespace diesel_vision_backend::arm;
    CameraTarget target;
    target.valid = true;
    target.stamp_s = 1.0;
    target.camera_target.translation() = Eigen::Vector3d(0, 0, .2);
    CameraTransformSample sample;
    sample.valid = true;
    sample.stamp_s = 1.0;
    sample.base_camera.translation() = Eigen::Vector3d(.1, 0, 0);
    const auto result = target_in_base(target, sample, 1.01, {.1, .005});
    ASSERT_TRUE(result);
    EXPECT_LT((result->translation() - Eigen::Vector3d(.1, 0, .2)).norm(), 1e-12);
    sample.stamp_s = .98;
    EXPECT_FALSE(target_in_base(target, sample, 1.01, {.1, .005}));
    sample.stamp_s = 1.0;
    EXPECT_FALSE(target_in_base(target, sample, 1.2, {.1, .005}));
    target.camera_target.linear()(0, 0) = -1;
    EXPECT_FALSE(target_in_base(target, sample, 1.01, {.1, .005}));
}
