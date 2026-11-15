#pragma once

#include <Eigen/Geometry>

namespace diesel_vision_backend::arm {

/// 几何角，不是电机编码器角；全部使用 rad。
struct LevelArmAngles {
    double base_yaw = 0.0;
    double upper_elevation = 0.0;   // 大臂相对水平面的仰角。
    double forearm_elevation = 0.0; // 小臂相对水平面的仰角，不是肘部夹角。
};

struct LevelArmGeometry {
    double upper_length_m = 0.149;
    double forearm_length_m = 0.149;
    // 暂以大臂根轴与yaw轴的交点为原点；实物轴偏置尚待测量。
};

struct LevelArmState {
    Eigen::Vector3d elbow_m;
    Eigen::Isometry3d base_endpoint;
};

/// 三主动自由度、末端理想保持水平的几何核心。
/// 只描述轴线简化模型：不含电机映射、限位、碰撞或夹爪安装偏置。
class LevelArmModel {
  public:
    explicit LevelArmModel(LevelArmGeometry geometry = {});
    LevelArmState forward(const LevelArmAngles& angles) const;

  private:
    LevelArmGeometry geometry_;
};

} // namespace diesel_vision_backend::arm
