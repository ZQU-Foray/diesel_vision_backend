#pragma once

#include "diesel_vision_backend/arm/target_angle_check.hpp"
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace diesel_vision_backend::arm {

/// 两个非相邻参考系之间的固定安装变换与基座系目标，单位m/rad。
struct TcpGoal {
    Eigen::Isometry3d base_tcp = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d endpoint_tcp = Eigen::Isometry3d::Identity();
};

/// 解析解返回靠近reference的连续角代表；不穷举多圈电机的所有等价解。
/// 不可达、不可实现朝向、轴心退化或非法输入返回空；position_tolerance_m为数值容差。
std::vector<LevelArmAngles> inverse_tcp(const LevelArmGeometry& geometry, const TcpGoal& goal,
                                        const LevelArmAngles& reference,
                                        double position_tolerance_m = 1e-8);

/// 位置雅可比：基座系TCP线速度 = J * [yaw速度,大臂绝对角速度,小臂绝对角速度]。
Eigen::Matrix3d tcp_position_jacobian(const LevelArmGeometry& geometry,
                                      const LevelArmAngles& angles,
                                      const Eigen::Vector3d& endpoint_tcp_translation);

enum class CollisionVerdict { Free, Collision, Unknown };
/// 调用方必须检查完整机构和环境；Free仅代表该检查器的模型，不是实机安全认证。
using CollisionCheck = std::function<CollisionVerdict(const LevelArmAngles&, const LevelArmState&)>;

struct MotionConfig {
    LevelArmGeometry geometry;
    MotorAngleMapping mapping;
    std::optional<TargetAngleLimits> limits;
    Eigen::Vector3d joint_speed_rad_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d joint_accel_rad_s2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d motor_speed_rad_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d motor_accel_rad_s2 = Eigen::Vector3d::Zero();
    double max_path_step_rad = 0.0; ///< 沿途采样每轴最大角间隔；有限采样不证明连续无碰撞。
    double min_position_singular_value_m = 0.0; ///< J最小奇异值下限，单位m/rad。
    double minimum_duration_s = 0.0;
    double start_speed_tolerance_rad_s = 0.0; ///< 本版仅接受静止起点。
    double cartesian_step_m = 0.0; ///< 直线段沿途采样间距；零值表示未配置直线能力。
    double tcp_speed_m_s = 0.0;
    double tcp_accel_m_s2 = 0.0;
};

enum class MotionStatus {
    ReadyForSimulation,
    InvalidConfiguration,
    InvalidInput,
    StartMoving,
    StartRejected,
    NoInverseSolution,
    NoFeasiblePath
};

struct CandidateReport {
    LevelArmAngles target;
    std::string reason; ///< ok或首个拒绝原因；采样下标与比例用于复现。
    std::size_t sample_index = 0;
    double path_fraction = 0.0;
    double min_singular_value_m = 0.0;
    double joint_travel_rad = 0.0;
};

struct TrajectoryPoint {
    LevelArmAngles joints;
    MotorAngles motors;
    Eigen::Vector3d joint_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d joint_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d motor_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d motor_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d tcp_m = Eigen::Vector3d::Zero();
};

/// 五次时间缩放轨迹：关节直线或固定TCP朝向的笛卡尔直线；不包含设备写操作。
class JointTrajectory {
  public:
    double duration_s() const;
    bool is_cartesian() const;
    /// t<0、非有限或无效对象抛invalid_argument；结束以后返回静止终点。
    TrajectoryPoint at(double elapsed_s) const;

  private:
    friend class ArmMotionPlanner;
    friend class ArmExecutionMonitor;
    MotionConfig config_;
    LevelArmAngles start_, target_;
    Eigen::Isometry3d endpoint_tcp_ = Eigen::Isometry3d::Identity();
    double duration_s_ = 0.0;
    bool cartesian_ = false;
    Eigen::Isometry3d cartesian_begin_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d cartesian_end_ = Eigen::Isometry3d::Identity();
};

struct MotionPlan {
    MotionStatus status = MotionStatus::InvalidConfiguration;
    std::string reason;
    std::vector<CandidateReport> candidates;
    std::optional<JointTrajectory> trajectory;
};

/// 离线规划模块：静止反馈→TCP IK→采样约束→排序→时间缩放。不接串口，不自动绕障。
class ArmMotionPlanner {
  public:
    ArmMotionPlanner(MotionConfig config, CollisionCheck collision);
    MotionPlan plan(const LevelArmAngles& current, const Eigen::Vector3d& current_velocity,
                    const TcpGoal& goal) const;
    /// 保持TCP朝向，沿基座系直线求同分支IK；失败不降级成关节弯线。
    MotionPlan plan_cartesian(const LevelArmAngles& current,
                              const Eigen::Vector3d& current_velocity, const TcpGoal& goal) const;

  private:
    MotionConfig config_;
    CollisionCheck collision_;
};

struct ArmFeedback {
    MotorAngles motors;
    Eigen::Vector3d motor_velocity = Eigen::Vector3d::Zero();
    double stamp_s = 0.0; ///< 与now同一时钟域，仿真用模拟时间。
    bool fault = false;
};

struct FeedbackPolicy {
    double max_age_s = 0.0;
    double joint_error_rad = 0.0;
    double tcp_error_m = 0.0;
    double stopped_speed_rad_s = 0.0;
    double settle_s = 0.0;
    double max_tracking_error_rad = 0.0;
    double execution_timeout_s = 0.0;
    double max_cartesian_error_m = 0.0; ///< 直线轨迹必须显式设置；按实际TCP监测。
};

enum class ExecutionStatus { Running, Settling, Arrived, Failed };
struct ExecutionReport {
    ExecutionStatus status = ExecutionStatus::Failed;
    std::string reason;
    /// 尚无有效反馈时为NaN，禁止把默认零坐标当作实测。
    Eigen::Vector3d actual_tcp_m =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
};

/// 单次轨迹监测；失败锁存，须重新规划/新建监测器，不把实际角覆盖为目标角。
class ArmExecutionMonitor {
  public:
    ArmExecutionMonitor(JointTrajectory trajectory, FeedbackPolicy policy, CollisionCheck collision,
                        double start_s);
    ExecutionReport update(const ArmFeedback& feedback, double now_s);

  private:
    JointTrajectory trajectory_;
    MotionConfig config_;
    FeedbackPolicy policy_;
    CollisionCheck collision_;
    Eigen::Isometry3d endpoint_tcp_;
    double start_s_, last_stamp_s_ = -1.0, last_now_s_ = -1.0;
    std::optional<double> settled_since_;
    bool failed_ = false;
};

} // namespace diesel_vision_backend::arm
