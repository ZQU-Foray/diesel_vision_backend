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
    // 手算姿态：两杆一上一下各30°，高度抵消，不把小臂角当肘部相对角。
    const auto opposed = model.forward({0, pi / 6, -pi / 6});
    require((opposed.base_endpoint.translation() - Eigen::Vector3d(.149 * std::sqrt(3.0), 0, 0))
                .norm() < 1e-12);
    // 非等长例子，数值来自测试模型而非实物：大臂竖直200mm、小臂向下100mm。
    LevelArmModel unequal({.2, .1});
    const auto folded = unequal.forward({0, pi / 2, -pi / 2});
    require((folded.base_endpoint.translation() - Eigen::Vector3d(0, 0, .1)).norm() < 1e-12);
    // 125组理想几何组合：检查两段长度、绕底座旋转的一致性和末端理想水平。
    // 这不是可达性、限位或碰撞验收；角度网格仅为数学回归输入。
    for (double yaw : {-pi, -pi / 2, 0.0, pi / 2, pi}) {
        for (double upper : {-pi / 2, -pi / 6, 0.0, pi / 6, pi / 2}) {
            for (double forearm : {-pi / 2, -pi / 6, 0.0, pi / 6, pi / 2}) {
                const auto state = model.forward({yaw, upper, forearm});
                const auto side = model.forward({0, upper, forearm});
                const Eigen::Matrix3d rotation =
                    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                require(std::abs(state.elbow_m.norm() - .149) < 1e-12);
                require(std::abs((state.base_endpoint.translation() - state.elbow_m).norm() -
                                 .149) < 1e-12);
                require((state.base_endpoint.translation() -
                         rotation * side.base_endpoint.translation())
                            .norm() < 1e-12);
                require((state.base_endpoint.linear() - rotation).norm() < 1e-12);
            }
        }
    }
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
    std::cout << "PASS: baseline geometry, opposed arms, unequal links, 125 posture combinations, "
                 "invalid angle, invalid length\n";
}
