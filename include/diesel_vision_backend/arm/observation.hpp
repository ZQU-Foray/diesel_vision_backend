#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <optional>

namespace diesel_vision_backend::arm {

/// 前端C++草案的后端适配输入；旋转/平移来自同一次PnP，不复用装甲板法向。
/// frame明确为camera；stamp_s须与相机变换样本处于同一时钟域。
struct CameraTarget {
    Eigen::Isometry3d camera_target = Eigen::Isometry3d::Identity();
    double stamp_s = 0.0;
    bool valid = false;
};

/// 外部根据实际关节状态和手眼参数提供；不可传入期望关节状态冒充实测。
struct CameraTransformSample {
    Eigen::Isometry3d base_camera = Eigen::Isometry3d::Identity();
    double stamp_s = 0.0;
    bool valid = false;
};

/// 显式注入时效与配对容差，单位s。无默认实机门限。
struct PairingPolicy {
    double max_age_s;
    double max_difference_s;
};

/// T_base_target = T_base_camera * T_camera_target；失败返回空，不伪造零位姿。
std::optional<Eigen::Isometry3d> target_in_base(const CameraTarget& target,
                                                const CameraTransformSample& sample, double now_s,
                                                PairingPolicy policy);

} // namespace diesel_vision_backend::arm
