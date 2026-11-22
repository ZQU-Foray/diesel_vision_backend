#include "diesel_vision_backend/arm/motion_pipeline.hpp"
#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace diesel_vision_backend::arm {
namespace {
constexpr double pi = 3.14159265358979323846;
// 数值/资源保护，不是实物安全参数；防止非法请求造成无界采样或溢出。
constexpr std::size_t max_path_samples = 100000;
Eigen::Vector3d vector(const LevelArmAngles& q) {
    return {q.base_yaw, q.upper_elevation, q.forearm_elevation};
}
Eigen::Vector3d vector(const MotorAngles& q) {
    return {q.base_motor, q.upper_motor, q.forearm_motor};
}
LevelArmAngles angles(const Eigen::Vector3d& q) {
    return {q[0], q[1], q[2]};
}
bool positive(double x) {
    return std::isfinite(x) && x > 0.0;
}
bool positive(const Eigen::Vector3d& x) {
    return x.allFinite() && (x.array() > 0.0).all();
}
bool pose_valid(const Eigen::Isometry3d& pose) {
    return pose.matrix().allFinite() &&
           (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() <
               1e-8 &&
           std::abs(pose.linear().determinant() - 1.0) < 1e-8 &&
           (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-8;
}

std::optional<LevelArmAngles> same_branch_ik(const LevelArmGeometry& geometry, const TcpGoal& goal,
                                             const LevelArmAngles& reference,
                                             const LevelArmAngles& branch) {
    const double branch_sign = std::sin(branch.forearm_elevation - branch.upper_elevation);
    const auto candidates = inverse_tcp(geometry, goal, reference);
    std::optional<LevelArmAngles> best;
    double distance = std::numeric_limits<double>::infinity();
    for (const auto& q : candidates) {
        if (std::sin(q.forearm_elevation - q.upper_elevation) * branch_sign <= 0)
            continue;
        const double cost = (vector(q) - vector(reference)).norm();
        if (cost < distance) {
            best = q;
            distance = cost;
        }
    }
    return best;
}

// 固定关节速度时TCP二阶运动项Jdot*qdot；包括yaw耦合与大小臂各自圆弧曲率。
Eigen::Vector3d acceleration_bias(const LevelArmGeometry& g, const LevelArmAngles& q,
                                  const Eigen::Vector3d& velocity, const Eigen::Vector3d& tool) {
    const Eigen::Vector3d tcp = LevelArmModel(g).forward(q).base_endpoint * tool;
    const double cosine = std::cos(q.base_yaw), sine = std::sin(q.base_yaw);
    const double radial_speed = -g.upper_length_m * std::sin(q.upper_elevation) * velocity[1] -
                                g.forearm_length_m * std::sin(q.forearm_elevation) * velocity[2];
    return Eigen::Vector3d(-tcp.x(), -tcp.y(), 0) * velocity[0] * velocity[0] +
           2 * velocity[0] * radial_speed * Eigen::Vector3d(-sine, cosine, 0) -
           g.upper_length_m * velocity[1] * velocity[1] *
               Eigen::Vector3d(std::cos(q.upper_elevation) * cosine,
                               std::cos(q.upper_elevation) * sine, std::sin(q.upper_elevation)) -
           g.forearm_length_m * velocity[2] * velocity[2] *
               Eigen::Vector3d(std::cos(q.forearm_elevation) * cosine,
                               std::cos(q.forearm_elevation) * sine, std::sin(q.forearm_elevation));
}
bool config_valid(const MotionConfig& c) {
    return positive(c.geometry.upper_length_m) && positive(c.geometry.forearm_length_m) &&
           c.mapping.is_configured() && c.limits && positive(c.joint_speed_rad_s) &&
           positive(c.joint_accel_rad_s2) && positive(c.motor_speed_rad_s) &&
           positive(c.motor_accel_rad_s2) && positive(c.max_path_step_rad) &&
           positive(c.min_position_singular_value_m) && positive(c.minimum_duration_s) &&
           positive(c.start_speed_tolerance_rad_s);
}
double singular_value(const MotionConfig& c, const LevelArmAngles& q, const Eigen::Vector3d& tool) {
    const Eigen::Matrix3d jacobian = tcp_position_jacobian(c.geometry, q, tool);
    if (!jacobian.allFinite())
        return std::numeric_limits<double>::quiet_NaN();
    return Eigen::JacobiSVD<Eigen::Matrix3d>(jacobian).singularValues().minCoeff();
}
std::string state_rejection(const MotionConfig& c, const CollisionCheck& collision,
                            const LevelArmAngles& q, const Eigen::Vector3d& tool,
                            double& smallest) {
    const auto limit = check_target_angles(q, c.mapping, c.limits);
    if (limit.status != TargetAngleStatus::WithinStaticLimits) {
        return "angle_check=" + std::to_string(static_cast<int>(limit.status)) +
               ",axis=" + std::to_string(limit.offending_axis);
    }
    smallest = singular_value(c, q, tool);
    if (!std::isfinite(smallest) || smallest < c.min_position_singular_value_m) {
        return "near_singularity";
    }
    if (!collision)
        return "collision_unconfigured";
    try {
        switch (collision(q, LevelArmModel(c.geometry).forward(q))) {
        case CollisionVerdict::Free:
            return {};
        case CollisionVerdict::Collision:
            return "collision";
        case CollisionVerdict::Unknown:
            return "collision_unknown";
        }
    } catch (...) {
        return "collision_checker_failed";
    }
    return "collision_unknown";
}
} // namespace

std::vector<LevelArmAngles> inverse_tcp(const LevelArmGeometry& g, const TcpGoal& goal,
                                        const LevelArmAngles& reference, double tolerance) {
    std::vector<LevelArmAngles> result;
    if (!positive(g.upper_length_m) || !positive(g.forearm_length_m) ||
        !pose_valid(goal.base_tcp) || !pose_valid(goal.endpoint_tcp) ||
        !vector(reference).allFinite() || !positive(tolerance))
        return result;
    const Eigen::Isometry3d endpoint = goal.base_tcp * goal.endpoint_tcp.inverse();
    const double yaw = std::atan2(endpoint.linear()(1, 0), endpoint.linear()(0, 0));
    const Eigen::Matrix3d yaw_rotation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // 水平末端只允许Rz(yaw)，不能忽略输入的roll/pitch或把位置目标伪装为任意姿态。
    if ((endpoint.linear() - yaw_rotation).norm() > 1e-8)
        return result;
    const Eigen::Vector3d local = yaw_rotation.transpose() * endpoint.translation();
    if (!local.allFinite() || std::abs(local.y()) > tolerance)
        return result;
    const double distance = std::hypot(local.x(), local.z());
    const double upper = g.upper_length_m, forearm = g.forearm_length_m;
    if (!std::isfinite(distance) || distance <= tolerance ||
        distance > upper + forearm + tolerance || distance < std::abs(upper - forearm) - tolerance)
        return result;
    const double cosine =
        (distance * distance - upper * upper - forearm * forearm) / (2.0 * upper * forearm);
    if (!std::isfinite(cosine))
        return result;
    for (double sign : {-1.0, 1.0}) {
        const double relative = sign * std::acos(std::clamp(cosine, -1.0, 1.0));
        const double upper_angle =
            std::atan2(local.z(), local.x()) -
            std::atan2(forearm * std::sin(relative), upper + forearm * std::cos(relative));
        Eigen::Vector3d q(yaw, upper_angle, upper_angle + relative);
        // 为每个几何分支选择靠近实测起点的连续角代表，不对执行中的轨迹做wrap。
        for (int axis = 0; axis < 3; ++axis)
            q[axis] += 2.0 * pi * std::round((vector(reference)[axis] - q[axis]) / (2.0 * pi));
        if (!q.allFinite())
            continue;
        const auto forward = LevelArmModel(g).forward(angles(q)).base_endpoint * goal.endpoint_tcp;
        if (!forward.matrix().allFinite() ||
            (forward.translation() - goal.base_tcp.translation()).norm() > tolerance ||
            (forward.linear() - goal.base_tcp.linear()).norm() > 1e-8)
            continue;
        if (!result.empty() && (q - vector(result.front())).norm() < 1e-8)
            continue;
        result.push_back(angles(q));
    }
    return result;
}

Eigen::Matrix3d tcp_position_jacobian(const LevelArmGeometry& g, const LevelArmAngles& q,
                                      const Eigen::Vector3d& tool) {
    const auto state = LevelArmModel(g).forward(q);
    const Eigen::Vector3d tcp = state.base_endpoint * tool;
    const double cosine = std::cos(q.base_yaw), sine = std::sin(q.base_yaw);
    Eigen::Matrix3d jacobian;
    jacobian.col(0) = Eigen::Vector3d(-tcp.y(), tcp.x(), 0.0);
    jacobian.col(1) = g.upper_length_m * Eigen::Vector3d(-std::sin(q.upper_elevation) * cosine,
                                                         -std::sin(q.upper_elevation) * sine,
                                                         std::cos(q.upper_elevation));
    jacobian.col(2) = g.forearm_length_m * Eigen::Vector3d(-std::sin(q.forearm_elevation) * cosine,
                                                           -std::sin(q.forearm_elevation) * sine,
                                                           std::cos(q.forearm_elevation));
    return jacobian;
}

ArmMotionPlanner::ArmMotionPlanner(MotionConfig config, CollisionCheck collision)
    : config_(std::move(config)), collision_(std::move(collision)) {
}

MotionPlan ArmMotionPlanner::plan(const LevelArmAngles& current,
                                  const Eigen::Vector3d& current_velocity,
                                  const TcpGoal& goal) const {
    MotionPlan output;
    if (!config_valid(config_)) {
        output.reason = "invalid_configuration";
        return output;
    }
    if (!vector(current).allFinite() || !current_velocity.allFinite() ||
        !pose_valid(goal.base_tcp) || !pose_valid(goal.endpoint_tcp)) {
        output.status = MotionStatus::InvalidInput;
        output.reason = "invalid_input";
        return output;
    }
    if (current_velocity.cwiseAbs().maxCoeff() > config_.start_speed_tolerance_rad_s) {
        output.status = MotionStatus::StartMoving;
        output.reason = "start_not_stationary";
        return output;
    }
    double smallest = 0.0;
    output.reason =
        state_rejection(config_, collision_, current, goal.endpoint_tcp.translation(), smallest);
    if (!output.reason.empty()) {
        output.status = MotionStatus::StartRejected;
        return output;
    }
    const auto candidates = inverse_tcp(config_.geometry, goal, current);
    if (candidates.empty()) {
        output.status = MotionStatus::NoInverseSolution;
        output.reason = "unreachable_or_orientation_incompatible";
        return output;
    }
    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto& target : candidates) {
        CandidateReport report;
        report.target = target;
        report.min_singular_value_m = std::numeric_limits<double>::infinity();
        const Eigen::Vector3d delta = vector(target) - vector(current);
        report.joint_travel_rad = delta.cwiseAbs().sum();
        const double required = std::ceil(delta.cwiseAbs().maxCoeff() / config_.max_path_step_rad);
        if (!std::isfinite(required) || required > max_path_samples - 1) {
            report.reason = "sampling_budget_exceeded";
        } else {
            const std::size_t intervals =
                std::max<std::size_t>(1, static_cast<std::size_t>(required));
            for (std::size_t index = 0; index <= intervals; ++index) {
                report.sample_index = index;
                report.path_fraction = static_cast<double>(index) / intervals;
                double value = 0.0;
                report.reason = state_rejection(
                    config_, collision_, angles(vector(current) + report.path_fraction * delta),
                    goal.endpoint_tcp.translation(), value);
                report.min_singular_value_m = std::min(report.min_singular_value_m, value);
                if (!report.reason.empty())
                    break;
            }
        }
        if (report.reason.empty()) {
            report.reason = "ok";
            if (report.joint_travel_rad < best_cost) {
                JointTrajectory trajectory;
                trajectory.config_ = config_;
                trajectory.start_ = current;
                trajectory.target_ = target;
                trajectory.endpoint_tcp_ = goal.endpoint_tcp;
                double duration = config_.minimum_duration_s;
                const auto start_motor = config_.mapping.to_motor_angles(current);
                const auto end_motor = config_.mapping.to_motor_angles(target);
                const Eigen::Vector3d motor_delta = vector(*end_motor) - vector(*start_motor);
                // s=10u³-15u⁴+6u⁵，max s'=15/8，max |s''|=10/sqrt(3)。
                const double peak_speed = 15.0 / 8.0, peak_accel = 10.0 / std::sqrt(3.0);
                for (int axis = 0; axis < 3; ++axis) {
                    duration = std::max(
                        {duration,
                         peak_speed * std::abs(delta[axis]) / config_.joint_speed_rad_s[axis],
                         std::sqrt(peak_accel * std::abs(delta[axis]) /
                                   config_.joint_accel_rad_s2[axis]),
                         peak_speed * std::abs(motor_delta[axis]) / config_.motor_speed_rad_s[axis],
                         std::sqrt(peak_accel * std::abs(motor_delta[axis]) /
                                   config_.motor_accel_rad_s2[axis])});
                }
                if (positive(duration) && positive(duration * duration)) {
                    trajectory.duration_s_ = duration;
                    output.trajectory = trajectory;
                    best_cost = report.joint_travel_rad;
                } else
                    report.reason = "duration_overflow";
            }
        }
        output.candidates.push_back(report);
    }
    output.status =
        output.trajectory ? MotionStatus::ReadyForSimulation : MotionStatus::NoFeasiblePath;
    output.reason = output.trajectory ? "sampled_path_ready_not_hardware_authorization"
                                      : "all_candidates_rejected";
    return output;
}

MotionPlan ArmMotionPlanner::plan_cartesian(const LevelArmAngles& current,
                                            const Eigen::Vector3d& current_velocity,
                                            const TcpGoal& goal) const {
    MotionPlan output;
    if (!config_valid(config_) || !positive(config_.cartesian_step_m) ||
        !positive(config_.tcp_speed_m_s) || !positive(config_.tcp_accel_m_s2)) {
        output.reason = "cartesian_configuration_invalid";
        return output;
    }
    if (!vector(current).allFinite() || !current_velocity.allFinite() ||
        !pose_valid(goal.base_tcp) || !pose_valid(goal.endpoint_tcp)) {
        output.status = MotionStatus::InvalidInput;
        output.reason = "invalid_input";
        return output;
    }
    if (current_velocity.cwiseAbs().maxCoeff() > config_.start_speed_tolerance_rad_s) {
        output.status = MotionStatus::StartMoving;
        output.reason = "start_not_stationary";
        return output;
    }
    const Eigen::Isometry3d begin =
        LevelArmModel(config_.geometry).forward(current).base_endpoint * goal.endpoint_tcp;
    if ((begin.linear() - goal.base_tcp.linear()).norm() > 1e-8) {
        output.status = MotionStatus::NoInverseSolution;
        output.reason = "cartesian_orientation_changed";
        return output;
    }
    const Eigen::Vector3d translation = goal.base_tcp.translation() - begin.translation();
    const double length = translation.norm();
    // 使用J最小奇异值门限给出保守的关节步长采样密度；不等于连续碰撞证明。
    const double step = std::min(config_.cartesian_step_m, config_.min_position_singular_value_m *
                                                               config_.max_path_step_rad * .5);
    const double required = std::ceil(length / step);
    if (!std::isfinite(required) || required > max_path_samples - 1) {
        output.reason = "sampling_budget_exceeded";
        return output;
    }
    const std::size_t intervals = std::max<std::size_t>(32, static_cast<std::size_t>(required));
    auto previous = current;
    CandidateReport report;
    report.min_singular_value_m = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i <= intervals; ++i) {
        report.sample_index = i;
        report.path_fraction = static_cast<double>(i) / intervals;
        TcpGoal point = goal;
        point.base_tcp.translation() = begin.translation() + report.path_fraction * translation;
        const auto q = same_branch_ik(config_.geometry, point, previous, current);
        if (!q) {
            report.reason = "cartesian_ik_discontinuous_or_unreachable";
            break;
        }
        if ((vector(*q) - vector(previous)).cwiseAbs().maxCoeff() >
            config_.max_path_step_rad + 1e-9) {
            report.reason = "cartesian_joint_step_exceeded";
            break;
        }
        double sigma = 0;
        report.reason =
            state_rejection(config_, collision_, *q, goal.endpoint_tcp.translation(), sigma);
        report.min_singular_value_m = std::min(report.min_singular_value_m, sigma);
        if (!report.reason.empty())
            break;
        report.joint_travel_rad += (vector(*q) - vector(previous)).cwiseAbs().sum();
        previous = *q;
    }
    report.target = previous;
    if (!report.reason.empty()) {
        output.status = MotionStatus::NoFeasiblePath;
        output.reason = report.reason;
        output.candidates.push_back(report);
        return output;
    }
    JointTrajectory trajectory;
    trajectory.config_ = config_;
    trajectory.start_ = current;
    trajectory.target_ = previous;
    trajectory.endpoint_tcp_ = goal.endpoint_tcp;
    trajectory.cartesian_ = true;
    trajectory.cartesian_begin_ = begin;
    trajectory.cartesian_end_ = goal.base_tcp;
    trajectory.duration_s_ = 1.0;
    double duration = config_.minimum_duration_s;
    // 按一秒归一化轨迹采样导数；1.1为数值峰值采样余量，不是实物能力保证。
    // 限位/碰撞沿路径等间隔检查，速度/加速度另按均匀时间检查，不能混用采样分布。
    try {
        for (std::size_t i = 0; i <= intervals; ++i) {
            const auto sample = trajectory.at(static_cast<double>(i) / intervals);
            for (int axis = 0; axis < 3; ++axis) {
                duration = std::max(
                    {duration,
                     std::abs(sample.joint_velocity[axis]) / config_.joint_speed_rad_s[axis],
                     std::sqrt(std::abs(sample.joint_acceleration[axis]) /
                               config_.joint_accel_rad_s2[axis]),
                     std::abs(sample.motor_velocity[axis]) / config_.motor_speed_rad_s[axis],
                     std::sqrt(std::abs(sample.motor_acceleration[axis]) /
                               config_.motor_accel_rad_s2[axis])});
            }
        }
    } catch (const std::exception&) {
        output.reason = "cartesian_derivative_failed";
        return output;
    }
    duration = std::max({duration, 1.875 * length / config_.tcp_speed_m_s,
                         std::sqrt((10 / std::sqrt(3.0)) * length / config_.tcp_accel_m_s2)}) *
               1.1;
    if (!positive(duration) || !positive(duration * duration)) {
        output.reason = "duration_overflow";
        return output;
    }
    trajectory.duration_s_ = duration;
    report.reason = "ok";
    output.candidates.push_back(report);
    output.trajectory = trajectory;
    output.status = MotionStatus::ReadyForSimulation;
    output.reason = "cartesian_sampled_ready_not_hardware_authorization";
    return output;
}

bool JointTrajectory::is_cartesian() const {
    return cartesian_;
}

double JointTrajectory::duration_s() const {
    return duration_s_;
}
TrajectoryPoint JointTrajectory::at(double elapsed) const {
    if (!positive(duration_s_) || !std::isfinite(elapsed) || elapsed < 0)
        throw std::invalid_argument("invalid trajectory or elapsed time");
    const double u = std::min(elapsed / duration_s_, 1.0);
    const double s = std::clamp(u * u * u * (10.0 + u * (-15.0 + 6.0 * u)), 0.0, 1.0);
    const double velocity = 30.0 * u * u * (1.0 - u) * (1.0 - u) / duration_s_;
    const double acceleration =
        60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (duration_s_ * duration_s_);
    const Eigen::Vector3d delta = vector(target_) - vector(start_);
    TrajectoryPoint point;
    point.joints = angles(vector(start_) + s * delta);
    point.joint_velocity = velocity * delta;
    point.joint_acceleration = acceleration * delta;
    if (cartesian_) {
        TcpGoal goal;
        goal.endpoint_tcp = endpoint_tcp_;
        goal.base_tcp = cartesian_end_;
        const Eigen::Vector3d path_delta =
            cartesian_end_.translation() - cartesian_begin_.translation();
        goal.base_tcp.translation() = cartesian_begin_.translation() + s * path_delta;
        const auto q = same_branch_ik(config_.geometry, goal, point.joints, start_);
        if (!q)
            throw std::runtime_error("cartesian IK branch lost");
        point.joints = *q;
        const Eigen::Matrix3d jacobian =
            tcp_position_jacobian(config_.geometry, *q, endpoint_tcp_.translation());
        Eigen::FullPivLU<Eigen::Matrix3d> solver(jacobian);
        if (!solver.isInvertible())
            throw std::runtime_error("cartesian Jacobian singular");
        const Eigen::Vector3d path_first = solver.solve(path_delta);
        const Eigen::Vector3d path_second = solver.solve(
            -acceleration_bias(config_.geometry, *q, path_first, endpoint_tcp_.translation()));
        point.joint_velocity = path_first * velocity;
        point.joint_acceleration = path_second * velocity * velocity + path_first * acceleration;
    }
    const auto motors = config_.mapping.to_motor_angles(point.joints);
    if (!motors)
        throw std::runtime_error("trajectory motor mapping failed");
    point.motors = *motors;
    const Eigen::Vector3d motor_delta = vector(*config_.mapping.to_motor_angles(target_)) -
                                        vector(*config_.mapping.to_motor_angles(start_));
    point.motor_velocity = velocity * motor_delta;
    point.motor_acceleration = acceleration * motor_delta;
    if (cartesian_) {
        const auto motor_velocity = config_.mapping.to_motor_velocity(point.joint_velocity);
        const auto motor_acceleration = config_.mapping.to_motor_velocity(point.joint_acceleration);
        if (!motor_velocity || !motor_acceleration)
            throw std::runtime_error("cartesian motor derivatives invalid");
        point.motor_velocity = *motor_velocity;
        point.motor_acceleration = *motor_acceleration;
    }
    point.tcp_m =
        (LevelArmModel(config_.geometry).forward(point.joints).base_endpoint * endpoint_tcp_)
            .translation();
    return point;
}

ArmExecutionMonitor::ArmExecutionMonitor(JointTrajectory trajectory, FeedbackPolicy policy,
                                         CollisionCheck collision, double start_s)
    : trajectory_(std::move(trajectory)), config_(trajectory_.config_), policy_(policy),
      collision_(std::move(collision)), endpoint_tcp_(trajectory_.endpoint_tcp_),
      start_s_(start_s) {
    failed_ = !config_valid(config_) || !pose_valid(endpoint_tcp_) || !std::isfinite(start_s_) ||
              start_s_ < 0 || !positive(trajectory_.duration_s()) || !positive(policy.max_age_s) ||
              !positive(policy.joint_error_rad) || !positive(policy.tcp_error_m) ||
              !positive(policy.stopped_speed_rad_s) || !positive(policy.settle_s) ||
              !positive(policy.max_tracking_error_rad) || !positive(policy.execution_timeout_s) ||
              policy.execution_timeout_s <= trajectory_.duration_s();
    if (trajectory_.is_cartesian() && !positive(policy_.max_cartesian_error_m))
        failed_ = true;
}

ExecutionReport ArmExecutionMonitor::update(const ArmFeedback& feedback, double now) {
    ExecutionReport report;
    auto fail = [&](const std::string& reason) {
        failed_ = true;
        report.reason = reason;
        return report;
    };
    if (failed_)
        return fail("failure_latched_or_invalid_monitor");
    if (feedback.fault)
        return fail("actuator_fault");
    if (!std::isfinite(now) || !std::isfinite(feedback.stamp_s) || now < start_s_ ||
        now <= last_now_s_ || feedback.stamp_s < start_s_ || feedback.stamp_s <= last_stamp_s_ ||
        feedback.stamp_s > now || now - feedback.stamp_s > policy_.max_age_s)
        return fail("feedback_time_invalid_or_stale");
    if (last_stamp_s_ >= 0 && feedback.stamp_s - last_stamp_s_ > policy_.max_age_s)
        return fail("feedback_gap_exceeded");
    last_now_s_ = now;
    last_stamp_s_ = feedback.stamp_s;
    const double elapsed = now - start_s_;
    if (elapsed > policy_.execution_timeout_s)
        return fail("execution_timeout");
    const auto actual = config_.mapping.to_joint_angles(feedback.motors);
    if (!actual || !feedback.motor_velocity.allFinite())
        return fail("invalid_feedback");
    report.actual_tcp_m =
        (LevelArmModel(config_.geometry).forward(*actual).base_endpoint * endpoint_tcp_)
            .translation();
    if (trajectory_.is_cartesian()) {
        const Eigen::Vector3d begin = trajectory_.cartesian_begin_.translation();
        const Eigen::Vector3d delta = trajectory_.cartesian_end_.translation() - begin;
        const double fraction =
            delta.squaredNorm() > 1e-20
                ? std::clamp((report.actual_tcp_m - begin).dot(delta) / delta.squaredNorm(), 0.0,
                             1.0)
                : 0;
        if ((report.actual_tcp_m - begin - fraction * delta).norm() > policy_.max_cartesian_error_m)
            return fail("actual_tcp_line_error_exceeded");
    }
    const auto expected = trajectory_.at(feedback.stamp_s - start_s_);
    double smallest = 0.0;
    const auto rejection =
        state_rejection(config_, collision_, *actual, endpoint_tcp_.translation(), smallest);
    if (!rejection.empty())
        return fail("feedback_" + rejection);
    if ((vector(*actual) - vector(expected.joints)).cwiseAbs().maxCoeff() >
        policy_.max_tracking_error_rad)
        return fail("tracking_error_exceeded");
    const auto joint_velocity = config_.mapping.to_joint_velocity(feedback.motor_velocity);
    if (!joint_velocity)
        return fail("velocity_mapping_failed");
    const Eigen::Vector3d speed = *joint_velocity;
    if ((speed.cwiseAbs().array() > config_.joint_speed_rad_s.array()).any() ||
        (feedback.motor_velocity.cwiseAbs().array() > config_.motor_speed_rad_s.array()).any())
        return fail("feedback_overspeed");
    const auto end = trajectory_.at(trajectory_.duration_s());
    const bool arrived =
        feedback.stamp_s - start_s_ >= trajectory_.duration_s() &&
        (vector(*actual) - vector(end.joints)).cwiseAbs().maxCoeff() <= policy_.joint_error_rad &&
        (report.actual_tcp_m - end.tcp_m).norm() <= policy_.tcp_error_m &&
        speed.cwiseAbs().maxCoeff() <= policy_.stopped_speed_rad_s;
    report.status = ExecutionStatus::Running;
    report.reason = "running";
    if (!arrived) {
        settled_since_.reset();
        return report;
    }
    // 用反馈采样时间计持续到位，不用接收时间提前宣告到达。
    if (!settled_since_)
        settled_since_ = feedback.stamp_s;
    report.status = feedback.stamp_s - *settled_since_ >= policy_.settle_s
                        ? ExecutionStatus::Arrived
                        : ExecutionStatus::Settling;
    report.reason = report.status == ExecutionStatus::Arrived ? "arrived" : "settling";
    return report;
}
} // namespace diesel_vision_backend::arm
