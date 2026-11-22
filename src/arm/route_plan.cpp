#include "diesel_vision_backend/arm/route_plan.hpp"
#include <cmath>

namespace diesel_vision_backend::arm {
RoutePlan plan_top_down_route(const MotionConfig& config, const CollisionCheck& collision,
                              const LevelArmAngles& start, const Eigen::Vector3d& velocity,
                              const TcpGoal& goal, double safe_height) {
    RoutePlan result;
    const auto state = LevelArmModel(config.geometry).forward(start);
    const Eigen::Vector3d initial_tcp = state.base_endpoint * goal.endpoint_tcp.translation();
    if (!std::isfinite(safe_height) || safe_height <= goal.base_tcp.translation().z() ||
        safe_height < initial_tcp.z() ||
        (goal.base_tcp.linear().col(2) + Eigen::Vector3d::UnitZ()).norm() > 1e-8) {
        result.attempts.push_back("invalid_top_down_geometry");
        return result;
    }
    TcpGoal lift = goal, above = goal;
    // 准备段允许关节空间运动；对准和下降必须为TCP直线。
    const double yaw = std::atan2(goal.base_tcp.linear()(1, 0), goal.base_tcp.linear()(0, 0));
    const double radius = initial_tcp.head<2>().norm();
    lift.base_tcp.translation() =
        Eigen::Vector3d(radius * std::cos(yaw), radius * std::sin(yaw), safe_height);
    above.base_tcp.translation().z() = safe_height;
    auto current = start;
    const std::vector<TcpGoal> goals{lift, above, goal};
    for (std::size_t i = 0; i < goals.size(); ++i) {
        ArmMotionPlanner planner(config, collision);
        const Eigen::Vector3d speed = i == 0 ? velocity : Eigen::Vector3d::Zero();
        auto plan = i == 0 ? planner.plan(current, speed, goals[i])
                           : planner.plan_cartesian(current, speed, goals[i]);
        if (!plan.trajectory) {
            result.attempts.push_back("top_down_segment=" + std::to_string(i) + ":" + plan.reason);
            result.segments.clear();
            result.motion_duration_s = 0;
            return result;
        }
        result.motion_duration_s += plan.trajectory->duration_s();
        current = plan.trajectory->at(plan.trajectory->duration_s()).joints;
        result.segments.push_back(*plan.trajectory);
    }
    result.selected_route = 0;
    result.attempts.push_back("top_down:lift_horizontal_down_ok");
    return result;
}
RoutePlan plan_route(const MotionConfig& config, const CollisionCheck& collision,
                     const LevelArmAngles& start, const Eigen::Vector3d& velocity,
                     const TcpGoal& goal, const std::vector<std::vector<TcpGoal>>& fallback,
                     bool cartesian_final) {
    RoutePlan result;
    // 有限资源保护，不是实物参数；禁止无界组合。
    if (fallback.size() > 64) {
        result.attempts.push_back("too_many_routes");
        return result;
    }
    for (std::size_t route = 0; route <= fallback.size(); ++route) {
        std::vector<TcpGoal> goals;
        if (route)
            goals = fallback[route - 1];
        if (goals.size() > 16) {
            result.attempts.push_back("too_many_waypoints");
            continue;
        }
        goals.push_back(goal);
        auto current = start;
        std::vector<JointTrajectory> segments;
        double duration = 0;
        bool accepted = true;
        for (std::size_t segment = 0; segment < goals.size(); ++segment) {
            // 同一工具贯穿路线；不允许候选中途偷偷改变TCP定义。
            if ((goals[segment].endpoint_tcp.matrix() - goal.endpoint_tcp.matrix()).norm() >
                1e-10) {
                result.attempts.push_back("tool_changed");
                accepted = false;
                break;
            }
            ArmMotionPlanner planner(config, collision);
            const Eigen::Vector3d current_velocity =
                segment == 0 ? velocity : Eigen::Vector3d::Zero();
            auto p = cartesian_final && segment + 1 == goals.size()
                         ? planner.plan_cartesian(current, current_velocity, goals[segment])
                         : planner.plan(current, current_velocity, goals[segment]);
            if (!p.trajectory) {
                std::string reason = p.reason;
                for (const auto& c : p.candidates)
                    reason += "/" + c.reason;
                result.attempts.push_back("route=" + std::to_string(route) +
                                          ",segment=" + std::to_string(segment) + ":" + reason);
                accepted = false;
                break;
            }
            duration += p.trajectory->duration_s();
            current = p.trajectory->at(p.trajectory->duration_s()).joints;
            segments.push_back(*p.trajectory);
        }
        if (accepted) {
            result.attempts.push_back("route=" + std::to_string(route) + ":ok");
            result.segments = std::move(segments);
            result.selected_route = static_cast<int>(route);
            result.motion_duration_s = duration;
            return result;
        }
    }
    return result;
}
} // namespace diesel_vision_backend::arm
