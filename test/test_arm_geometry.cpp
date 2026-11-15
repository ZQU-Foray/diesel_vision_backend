#include "diesel_vision_backend/arm/level_arm.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
    using namespace diesel_vision_backend::arm;
    const auto require = [](bool ok) {
        if (!ok)
            throw std::runtime_error("level arm check failed");
    };
    const double pi = std::acos(-1.0);
    LevelArmModel model;
    const auto straight = model.forward({0, 0, 0});
    require((straight.base_endpoint.translation() - Eigen::Vector3d(.298, 0, 0)).norm() < 1e-12);
    const auto yaw90 = model.forward({pi / 2, 0, 0});
    require((yaw90.base_endpoint.translation() - Eigen::Vector3d(0, .298, 0)).norm() < 1e-12);
    const auto bent = model.forward({0, pi / 2, 0});
    require((bent.base_endpoint.translation() - Eigen::Vector3d(.149, 0, .149)).norm() < 1e-12);
    require((bent.base_endpoint.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-12);
    bool rejected = false;
    try {
        model.forward({0, std::numeric_limits<double>::quiet_NaN(), 0});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected);
    rejected = false;
    try {
        LevelArmModel invalid({0, .149});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected);
    std::cout << "PASS: straight, yaw90, bent, level orientation, invalid angle, invalid length\n";
}
