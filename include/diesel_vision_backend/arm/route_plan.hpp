#pragma once
#include "diesel_vision_backend/arm/motion_pipeline.hpp"

namespace diesel_vision_backend::arm {
struct RoutePlan {
    std::vector<JointTrajectory> segments;
    std::vector<std::string> attempts;
    int selected_route = -1;        ///< 0为直接路线，1..为调用方经过点候选。
    double motion_duration_s = 0.0; ///< 不含段间实际到位等待。
};
/// 先直接规划；失败后逐个尝试经过点序列。每段检查，全部通过才返回；无绕行完备性保证。
/// fallback中只填中途TCP位姿，函数自动在每条序列末尾添加最终goal。
/// cartesian_final=true时末段必须保持TCP朝向并走直线，不允许失败后退成关节弯线。
RoutePlan plan_route(const MotionConfig& config, const CollisionCheck& collision,
                     const LevelArmAngles& start, const Eigen::Vector3d& velocity,
                     const TcpGoal& goal, const std::vector<std::vector<TcpGoal>>& fallback,
                     bool cartesian_final = false);
/// 顶抓：准备到安全高度→定高水平对准→固定x/y下降；失败不退化成斜线。
/// safe_height为基座系TCP高度，必须高于目标与起点；工具局部+Z必须朝下。
RoutePlan plan_top_down_route(const MotionConfig& config, const CollisionCheck& collision,
                              const LevelArmAngles& start, const Eigen::Vector3d& velocity,
                              const TcpGoal& goal, double safe_height);
} // namespace diesel_vision_backend::arm
