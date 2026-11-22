// 仿真进程适配入口：正式C++规划/监测，stdin/stdout逐步交互；不包含硬件写操作。
#include "diesel_vision_backend/arm/motion_pipeline.hpp"
#include "diesel_vision_backend/arm/observation.hpp"
#include "diesel_vision_backend/arm/route_plan.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace diesel_vision_backend::arm;
namespace {
double segment_distance(const Eigen::Vector3d& point, const Eigen::Vector3d& begin,
                        const Eigen::Vector3d& end) {
    const Eigen::Vector3d delta = end - begin;
    const double length_squared = delta.squaredNorm();
    const double fraction =
        length_squared > 0 ? std::clamp((point - begin).dot(delta) / length_squared, 0., 1.) : 0.;
    return (point - begin - fraction * delta).norm();
}
} // namespace
int main(int argc, char** argv) {
    try {
        // 明确是合成教学参数：149mm来自用户说明，其余不是实测、不可用于实物。
        MotionConfig c;
        MotorAngleCalibration calibration;
        calibration.motor_to_joint << 1, 0, 0, 0, 1, 0, 0, 1, 1;
        c.mapping = MotorAngleMapping(calibration); // MuJoCo电机3为相对肘角，几何小臂角=大臂+肘。
        TargetAngleLimits limits;
        limits.joint_min_rad = Eigen::Vector3d(-1.4, .05, -1.4);
        limits.joint_max_rad = Eigen::Vector3d(1.4, 2.7, 1.4);
        limits.motor_min_rad = Eigen::Vector3d(-1.4, .05, -2.8);
        limits.motor_max_rad = Eigen::Vector3d(1.4, 2.7, 2.8);
        c.limits = limits;
        c.joint_speed_rad_s = c.motor_speed_rad_s = Eigen::Vector3d::Constant(.6);
        c.joint_accel_rad_s2 = c.motor_accel_rad_s2 = Eigen::Vector3d::Constant(1.2);
        c.max_path_step_rad = .005;
        c.min_position_singular_value_m = .003;
        c.minimum_duration_s = .5;
        c.start_speed_tolerance_rad_s = .01;
        c.cartesian_step_m = .0005;
        c.tcp_speed_m_s = .04;
        c.tcp_accel_m_s2 = .08;
        const LevelArmAngles start{0, 1.0471975511965976, -.3490658503988659};
        TcpGoal goal;
        goal.base_tcp.translation() = Eigen::Vector3d(.20, 0, .10);
        goal.endpoint_tcp.translation() = Eigen::Vector3d(0, 0, -.025);
        const Eigen::Matrix3d downward =
            Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitX()).toRotationMatrix();
        goal.endpoint_tcp.linear() = downward; // 合成工具长25mm，局部+Z为向下接近方向。
        if (argc != 1 && argc != 5 && argc != 6)
            throw std::invalid_argument("usage: bridge [x y z yaw [block]]");
        if (argc >= 5) {
            goal.base_tcp.translation() =
                Eigen::Vector3d(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
            goal.base_tcp.linear() =
                Eigen::AngleAxisd(std::stod(argv[4]), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        }
        const std::string mode = argc == 6 ? argv[5] : "direct";
        const bool detour = mode.find("detour") != std::string::npos;
        const bool block = mode.find("block") != std::string::npos || detour;
        if (mode.find("camera") != std::string::npos) {
            // 图像测量和相机外参分开输入；同一模拟采样时刻，不接物体真值。
            CameraTarget observation;
            CameraTransformSample camera;
            std::string tag;
            double now;
            std::cin >> tag >> now >> observation.stamp_s >> camera.stamp_s;
            for (int i = 0; i < 3; ++i)
                std::cin >> observation.camera_target.translation()[i];
            for (int r = 0; r < 3; ++r)
                for (int col = 0; col < 3; ++col)
                    std::cin >> observation.camera_target.linear()(r, col);
            for (int i = 0; i < 3; ++i)
                std::cin >> camera.base_camera.translation()[i];
            for (int r = 0; r < 3; ++r)
                for (int col = 0; col < 3; ++col)
                    std::cin >> camera.base_camera.linear()(r, col);
            if (!std::cin || tag != "CAMERA")
                throw std::invalid_argument("invalid camera observation");
            std::string remainder;
            std::getline(std::cin, remainder);
            observation.valid = camera.valid = true;
            const auto object = target_in_base(observation, camera, now, {.05, .005});
            if (!object)
                throw std::invalid_argument("camera pairing rejected");
            Eigen::Isometry3d object_tcp = Eigen::Isometry3d::Identity();
            object_tcp.linear() =
                Eigen::AngleAxisd(-1.5707963267948966, Eigen::Vector3d::UnitX()).toRotationMatrix();
            object_tcp.translation() =
                Eigen::Vector3d(0, 0, -.015); // 标记板位于期望TCP前方15mm，仿真任务配置。
            goal.base_tcp = *object * object_tcp;
            const double yaw =
                std::atan2(goal.base_tcp.linear()(1, 0), goal.base_tcp.linear()(0, 0));
            const Eigen::Matrix3d horizontal =
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            if ((goal.base_tcp.linear() - horizontal).norm() > .15)
                throw std::invalid_argument("observed orientation incompatible");
            // 水平工具任务只消费观测yaw；小roll/pitch测量误差不是可执行自由度。
            goal.base_tcp.linear() = horizontal;
            // 三自由度只允许径向位置与yaw一致；小测量侧偏可容忍2mm后投影，否则拒绝。
            const Eigen::Vector3d local = horizontal.transpose() * goal.base_tcp.translation();
            if (std::abs(local.y()) > .002)
                throw std::invalid_argument("observed lateral mismatch");
            goal.base_tcp.translation() = horizontal * Eigen::Vector3d(local.x(), 0, local.z());
            std::cerr << "CAMERA基座TCP=" << goal.base_tcp.translation().transpose() << "\n";
        }
        goal.base_tcp.linear() = goal.base_tcp.linear() * downward;
        const Eigen::Vector3d obstacle =
            detour ? Eigen::Vector3d(.20, 0, .035) : Eigen::Vector3d(.18, 0, .10);
        CollisionCheck collision = [&](const auto&, const LevelArmState& state) {
            const Eigen::Vector3d root = Eigen::Vector3d::Zero();
            const Eigen::Vector3d end = state.base_endpoint.translation();
            const Eigen::Vector3d tcp = state.base_endpoint * goal.endpoint_tcp.translation();
            // 简化杆胶囊半径8mm、工具6mm、额外余量2mm；地面在基座原点下方30mm。
            if (std::min({root.z(), state.elbow_m.z(), end.z(), tcp.z()}) - .010 <= -.030)
                return CollisionVerdict::Collision;
            if (block && std::min({segment_distance(obstacle, root, state.elbow_m),
                                   segment_distance(obstacle, state.elbow_m, end),
                                   segment_distance(obstacle, end, tcp)}) <= (detour ? .014 : .032))
                return CollisionVerdict::Collision;
            // 两杆共享肘关节，未验证完整关节壳/底座/平行连杆/线缆自碰撞，不能用于实机。
            return CollisionVerdict::Free;
        };
        // 140mm为合成场景安全高度；顶抓失败不退回旧的斜向末段。
        const auto p =
            detour ? plan_top_down_route(c, collision, start, Eigen::Vector3d::Zero(), goal, .14)
                   : plan_route(c, collision, start, Eigen::Vector3d::Zero(), goal, {});
        std::cout << std::setprecision(17);
        for (const auto& r : p.attempts)
            std::cerr << "路线候选: " << r << "\n";
        if (p.segments.empty()) {
            std::cout << "REJECT no_feasible_route" << std::endl;
            return 2;
        }
        std::size_t segment = 0;
        double segment_start = 0;
        FeedbackPolicy policy{.02, .01, .002, .008, .25, .25, p.motion_duration_s + 4.0};
        policy.max_cartesian_error_m = .001; // 1mm为当前理想仿真末段横向误差验收值，不是实机指标。
        ArmExecutionMonitor monitor(p.segments[0], policy, collision, 0);
        std::cerr << "SELECT route=" << p.selected_route << " segments=" << p.segments.size()
                  << "\n";
        std::cout << "PLAN " << p.motion_duration_s << " " << start.base_yaw << " "
                  << start.upper_elevation << " " << start.forearm_elevation << " "
                  << p.selected_route << " " << p.segments.size() << std::endl;
        std::string line;
        while (std::getline(std::cin, line)) {
            std::istringstream input(line);
            std::string command;
            input >> command;
            if (command == "SAMPLE") {
                double elapsed;
                if (!(input >> elapsed))
                    throw std::invalid_argument("invalid SAMPLE");
                const auto point = p.segments[segment].at(std::max(0.0, elapsed - segment_start));
                const auto begin = p.segments[segment].at(0);
                const auto end = p.segments[segment].at(p.segments[segment].duration_s());
                std::cout << "POINT " << point.motors.base_motor << " " << point.motors.upper_motor
                          << " " << point.motors.forearm_motor << " " << point.tcp_m.transpose()
                          << " " << segment << " " << p.segments[segment].is_cartesian() << " "
                          << begin.tcp_m.transpose() << " " << end.tcp_m.transpose() << std::endl;
            } else if (command == "FEEDBACK") {
                ArmFeedback f;
                double now;
                int fault;
                if (!(input >> now >> f.stamp_s >> f.motors.base_motor >> f.motors.upper_motor >>
                      f.motors.forearm_motor >> f.motor_velocity[0] >> f.motor_velocity[1] >>
                      f.motor_velocity[2] >> fault))
                    throw std::invalid_argument("invalid FEEDBACK");
                f.fault = fault != 0;
                auto report = monitor.update(f, now);
                if (report.status == ExecutionStatus::Arrived && segment + 1 < p.segments.size()) {
                    ++segment;
                    segment_start = now;
                    monitor = ArmExecutionMonitor(p.segments[segment], policy, collision, now);
                    report.status = ExecutionStatus::Running;
                    report.reason = "next_segment=" + std::to_string(segment);
                }
                std::cout << "STATE " << static_cast<int>(report.status) << " "
                          << report.actual_tcp_m.transpose() << " " << report.reason << std::endl;
            } else
                throw std::invalid_argument("unknown command");
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
    return 0;
}
