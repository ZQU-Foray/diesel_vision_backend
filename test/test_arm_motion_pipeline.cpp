#include "diesel_vision_backend/arm/motion_pipeline.hpp"
#include "diesel_vision_backend/arm/route_plan.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace diesel_vision_backend::arm;

namespace {
constexpr double pi = 3.14159265358979323846;
MotionConfig config() {
    MotionConfig c;
    MotorAngleCalibration mapping;
    mapping.motor_to_joint = Eigen::Matrix3d::Identity();
    c.mapping = MotorAngleMapping(mapping);
    TargetAngleLimits limits;
    limits.joint_min_rad = limits.motor_min_rad = Eigen::Vector3d::Constant(-3.0);
    limits.joint_max_rad = limits.motor_max_rad = Eigen::Vector3d::Constant(3.0);
    c.limits = limits;
    c.joint_speed_rad_s = c.motor_speed_rad_s = Eigen::Vector3d::Constant(0.6);
    c.joint_accel_rad_s2 = c.motor_accel_rad_s2 = Eigen::Vector3d::Constant(1.2);
    c.max_path_step_rad = .005;
    c.min_position_singular_value_m = .003;
    c.minimum_duration_s = .2;
    c.start_speed_tolerance_rad_s = .01;
    c.cartesian_step_m = .0005;
    c.tcp_speed_m_s = .04;
    c.tcp_accel_m_s2 = .08;
    return c;
}
const LevelArmAngles start{0, 1.0, -.4};
CollisionVerdict free_space(const LevelArmAngles&, const LevelArmState&);
TEST(ArmTopDown, HorizontalThenVerticalAndNoDiagonalFallback) {
    TcpGoal target;
    target.endpoint_tcp.translation() = Eigen::Vector3d(0, 0, -.025);
    target.endpoint_tcp.linear() =
        Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitX()).toRotationMatrix();
    target.base_tcp.linear() = target.endpoint_tcp.linear();
    target.base_tcp.translation() = Eigen::Vector3d(.18, 0, .04);
    const auto route =
        plan_top_down_route(config(), free_space, start, Eigen::Vector3d::Zero(), target, .14);
    ASSERT_EQ(route.segments.size(), 3u);
    for (int i = 1; i <= 2; ++i) {
        ASSERT_TRUE(route.segments[i].is_cartesian());
        const auto& trajectory = route.segments[i];
        for (int sample = 0; sample <= 100; ++sample) {
            const auto point = trajectory.at(trajectory.duration_s() * sample / 100.);
            if (i == 1)
                EXPECT_NEAR(point.tcp_m.z(), .14, 1e-8);
            else {
                EXPECT_NEAR(point.tcp_m.x(), .18, 1e-8);
                EXPECT_NEAR(point.tcp_m.y(), 0, 1e-8);
                EXPECT_GE(point.tcp_m.z(), .04 - 1e-8);
            }
        }
    }
    EXPECT_TRUE(
        plan_top_down_route(config(), free_space, start, Eigen::Vector3d::Zero(), target, .03)
            .segments.empty());
    auto blocked = [](const LevelArmAngles&, const LevelArmState&) {
        return CollisionVerdict::Collision;
    };
    EXPECT_TRUE(plan_top_down_route(config(), blocked, start, Eigen::Vector3d::Zero(), target, .14)
                    .segments.empty());
}
TcpGoal goal(const LevelArmAngles& q) {
    TcpGoal g;
    g.endpoint_tcp.translation() = Eigen::Vector3d(.025, .01, .005);
    g.base_tcp = LevelArmModel().forward(q).base_endpoint * g.endpoint_tcp;
    return g;
}
CollisionVerdict free_space(const LevelArmAngles&, const LevelArmState&) {
    return CollisionVerdict::Free;
}
MotionPlan plan() {
    return ArmMotionPlanner(config(), free_space)
        .plan(start, Eigen::Vector3d::Zero(), goal({.3, .8, -.5}));
}
FeedbackPolicy policy(double duration) {
    return {.05, .005, .001, .01, .1, .3, duration + 2};
}
ArmFeedback feedback(const JointTrajectory& t, double stamp) {
    auto point = t.at(stamp);
    return {point.motors, point.motor_velocity, stamp, false};
}
} // namespace

TEST(ArmInverse, RoundTripMultipleBranchesAndToolOffset) {
    for (double yaw : {-.7, 0., .8}) {
        for (double upper : {.4, .9, 1.2}) {
            const auto g = goal({yaw, upper, -.5});
            auto candidates = inverse_tcp({}, g, {yaw, upper, -.5});
            ASSERT_EQ(candidates.size(), 2u);
            for (auto q : candidates) {
                const Eigen::Isometry3d tcp =
                    LevelArmModel().forward(q).base_endpoint * g.endpoint_tcp;
                EXPECT_LT((tcp.matrix() - g.base_tcp.matrix()).norm(), 1e-8);
            }
        }
    }
}
TEST(ArmInverse, RejectsUnreachableOrientationAndInvalidInput) {
    auto g = goal(start);
    g.base_tcp.translation().x() = 1;
    EXPECT_TRUE(inverse_tcp({}, g, start).empty());
    g = goal(start);
    g.base_tcp.linear() *= Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
    EXPECT_TRUE(inverse_tcp({}, g, start).empty());
    g.base_tcp.translation().z() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(inverse_tcp({}, g, start).empty());
}
TEST(ArmJacobian, MatchesFiniteDifferencesIncludingToolOffset) {
    const Eigen::Vector3d tool(.025, .01, .005);
    const auto j = tcp_position_jacobian({}, start, tool);
    for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d q(start.base_yaw, start.upper_elevation, start.forearm_elevation);
        q[axis] += 1e-6;
        const Eigen::Vector3d before = LevelArmModel().forward(start).base_endpoint * tool;
        const Eigen::Vector3d after =
            LevelArmModel().forward({q[0], q[1], q[2]}).base_endpoint * tool;
        EXPECT_LT(((after - before) / 1e-6 - j.col(axis)).norm(), 1e-6);
    }
}

TEST(ArmVelocityMapping, CouplingWithoutZeroOffsetAndInvalidValues) {
    MotorAngleCalibration calibration;
    calibration.motor_to_joint << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    calibration.motor_zero_rad = Eigen::Vector3d(10, 20, 30);
    calibration.joint_reference_rad = Eigen::Vector3d(2, 3, 4);
    MotorAngleMapping mapping(calibration);
    auto velocity = mapping.to_joint_velocity(Eigen::Vector3d(1, 2, 3));
    ASSERT_TRUE(velocity);
    EXPECT_LT((*velocity - Eigen::Vector3d(1, 2, 5)).norm(), 1e-12);
    EXPECT_FALSE(MotorAngleMapping().to_joint_velocity(Eigen::Vector3d::Zero()));
    EXPECT_FALSE(mapping.to_joint_velocity(
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())));
}

TEST(ArmRoute, DirectFirstAndWaypointsOnlyWhenNeeded) {
    const auto target = goal({.3, .8, -.5});
    const auto via = goal({0, .6, -.7});
    auto direct = plan_route(config(), free_space, start, Eigen::Vector3d::Zero(), target, {{via}});
    EXPECT_EQ(direct.selected_route, 0);
    EXPECT_EQ(direct.segments.size(), 1u);
    CollisionCheck obstacle = [](const auto& q, const auto&) {
        Eigen::Vector3d offset(q.base_yaw - .15, q.upper_elevation - .9, q.forearm_elevation + .45);
        return offset.norm() < .05 ? CollisionVerdict::Collision : CollisionVerdict::Free;
    };
    auto detour = plan_route(config(), obstacle, start, Eigen::Vector3d::Zero(), target, {{via}});
    ASSERT_EQ(detour.selected_route, 1);
    ASSERT_EQ(detour.segments.size(), 2u);
    EXPECT_NE(detour.attempts[0].find("collision"), std::string::npos);
    EXPECT_LT(
        (detour.segments[0].at(detour.segments[0].duration_s()).tcp_m - via.base_tcp.translation())
            .norm(),
        1e-8);
}

TEST(ArmCartesianRegression, FinalRouteMustFollowTcpLineNotJointChord) {
    auto route = plan_route(config(), free_space, start, Eigen::Vector3d::Zero(),
                            goal({0, .6, -.9}), {}, true);
    ASSERT_EQ(route.segments.size(), 1u);
    const auto& trajectory = route.segments.front();
    const Eigen::Vector3d begin = trajectory.at(0).tcp_m;
    const Eigen::Vector3d end = trajectory.at(trajectory.duration_s()).tcp_m;
    const Eigen::Vector3d direction = (end - begin).normalized();
    double maximum = 0;
    for (int i = 0; i <= 1000; ++i) {
        const Eigen::Vector3d offset =
            trajectory.at(trajectory.duration_s() * i / 1000.).tcp_m - begin;
        maximum = std::max(maximum, (offset - direction * offset.dot(direction)).norm());
    }
    EXPECT_LT(maximum, 1e-6) << "TCP straight-line max error (m)";
}

TEST(ArmCartesian, DerivativesMatchFiniteDifferencesAndAllSpeedAccelerationLimits) {
    auto c = config();
    MotorAngleCalibration mapping;
    mapping.motor_to_joint << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    c.mapping = MotorAngleMapping(mapping);
    c.motor_speed_rad_s = Eigen::Vector3d::Constant(.2);
    auto p = ArmMotionPlanner(c, free_space)
                 .plan_cartesian(start, Eigen::Vector3d::Zero(), goal({0, .6, -.9}));
    ASSERT_TRUE(p.trajectory) << p.reason;
    const auto& trajectory = *p.trajectory;
    const auto qvector = [](const auto& q) {
        return Eigen::Vector3d(q.base_yaw, q.upper_elevation, q.forearm_elevation);
    };
    const Eigen::Vector3d tool = goal(start).endpoint_tcp.translation();
    for (int i = 1; i < 1000; ++i) {
        const double time = trajectory.duration_s() * i / 1000.;
        const double h = 1e-5;
        const auto sample = trajectory.at(time);
        const auto before = trajectory.at(time - h), after = trajectory.at(time + h);
        EXPECT_LT(
            ((qvector(after.joints) - qvector(before.joints)) / (2 * h) - sample.joint_velocity)
                .norm(),
            1e-6);
        EXPECT_LT(
            ((after.joint_velocity - before.joint_velocity) / (2 * h) - sample.joint_acceleration)
                .norm(),
            1e-6);
        EXPECT_LT(((after.tcp_m - before.tcp_m) / (2 * h) -
                   tcp_position_jacobian(c.geometry, sample.joints, tool) * sample.joint_velocity)
                      .norm(),
                  1e-6);
        EXPECT_TRUE(
            (sample.joint_velocity.cwiseAbs().array() <= c.joint_speed_rad_s.array() + 1e-10)
                .all());
        EXPECT_TRUE(
            (sample.joint_acceleration.cwiseAbs().array() <= c.joint_accel_rad_s2.array() + 1e-10)
                .all());
        EXPECT_TRUE(
            (sample.motor_velocity.cwiseAbs().array() <= c.motor_speed_rad_s.array() + 1e-10)
                .all());
        EXPECT_TRUE(
            (sample.motor_acceleration.cwiseAbs().array() <= c.motor_accel_rad_s2.array() + 1e-10)
                .all());
        EXPECT_LT(
            (tcp_position_jacobian(c.geometry, sample.joints, tool) * sample.joint_velocity).norm(),
            c.tcp_speed_m_s);
        EXPECT_LT(((after.tcp_m - 2 * sample.tcp_m + before.tcp_m) / (h * h)).norm(),
                  c.tcp_accel_m_s2 + 1e-5);
        EXPECT_LT(std::sin(sample.joints.forearm_elevation - sample.joints.upper_elevation), 0);
    }
    EXPECT_LT(trajectory.at(0).joint_velocity.norm(), 1e-12);
    EXPECT_LT(trajectory.at(trajectory.duration_s()).joint_acceleration.norm(), 1e-12);
}

TEST(ArmCartesian, RejectsOrientationCollisionSingularityAndUnconfiguredLimits) {
    auto c = config();
    auto orientation = ArmMotionPlanner(c, free_space)
                           .plan_cartesian(start, Eigen::Vector3d::Zero(), goal({.1, .6, -.9}));
    EXPECT_FALSE(orientation.trajectory);
    EXPECT_EQ(orientation.reason, "cartesian_orientation_changed");
    auto collision = ArmMotionPlanner(c, [](auto&, auto&) {
                         return CollisionVerdict::Unknown;
                     }).plan_cartesian(start, Eigen::Vector3d::Zero(), goal({0, .6, -.9}));
    EXPECT_FALSE(collision.trajectory);
    EXPECT_FALSE(ArmMotionPlanner(c, free_space)
                     .plan_cartesian(start, Eigen::Vector3d::Zero(), goal({0, .3, .3}))
                     .trajectory);
    c.cartesian_step_m = 0;
    EXPECT_FALSE(ArmMotionPlanner(c, free_space)
                     .plan_cartesian(start, Eigen::Vector3d::Zero(), goal({0, .6, -.9}))
                     .trajectory);
}

TEST(ArmCartesian, ActualLateralErrorIsNotAllowedToHideBehindArrival) {
    auto c = config();
    auto p = ArmMotionPlanner(c, free_space)
                 .plan_cartesian(start, Eigen::Vector3d::Zero(), goal({0, .6, -.9}));
    ASSERT_TRUE(p.trajectory);
    auto limits = policy(p.trajectory->duration_s());
    limits.max_cartesian_error_m = .0001;
    ArmExecutionMonitor monitor(*p.trajectory, limits, free_space, 0);
    auto state = feedback(*p.trajectory, 0);
    state.motors.base_motor += .01;
    EXPECT_EQ(monitor.update(state, 0).reason, "actual_tcp_line_error_exceeded");
}

TEST(ArmRoute, EverySegmentAndToolDefinitionMustPass) {
    auto via = goal({0, .6, -.7});
    via.base_tcp.translation().x() = 1;
    auto rejected = plan_route(config(), [](auto&, auto&) { return CollisionVerdict::Collision; },
                               start, Eigen::Vector3d::Zero(), goal({.3, .8, -.5}), {{via}});
    EXPECT_TRUE(rejected.segments.empty());
    EXPECT_EQ(rejected.selected_route, -1);
    auto changed = goal({0, .6, -.7});
    changed.endpoint_tcp.translation().x() += .01;
    auto tool = plan_route(config(),
                           [](const auto& q, const auto&) {
                               return q.base_yaw > .1 && q.base_yaw < .2
                                          ? CollisionVerdict::Collision
                                          : CollisionVerdict::Free;
                           },
                           start, Eigen::Vector3d::Zero(), goal({.3, .8, -.5}), {{changed}});
    EXPECT_TRUE(tool.segments.empty());
    EXPECT_EQ(tool.attempts.back(), "tool_changed");
}
TEST(ArmPlanner, UnknownCollisionAndMovingStartFailClosed) {
    EXPECT_FALSE(ArmMotionPlanner(config(), {})
                     .plan(start, Eigen::Vector3d::Zero(), goal(start))
                     .trajectory);
    EXPECT_FALSE(ArmMotionPlanner(config(), [](auto&, auto&) { return CollisionVerdict::Unknown; })
                     .plan(start, Eigen::Vector3d::Zero(), goal(start))
                     .trajectory);
    EXPECT_EQ(ArmMotionPlanner(config(), free_space)
                  .plan(start, Eigen::Vector3d(.02, 0, 0), goal(start))
                  .status,
              MotionStatus::StartMoving);
    EXPECT_FALSE(ArmMotionPlanner({}, free_space)
                     .plan(start, Eigen::Vector3d::Zero(), goal(start))
                     .trajectory);
}
TEST(ArmPlanner, CollisionInMiddleRejectsEvenWithSafeEndpoints) {
    const auto p = ArmMotionPlanner(config(), [](const auto& q, const auto&) {
                       return q.base_yaw > .1 && q.base_yaw < .2 ? CollisionVerdict::Collision
                                                                 : CollisionVerdict::Free;
                   }).plan(start, Eigen::Vector3d::Zero(), goal({.3, .8, -.5}));
    ASSERT_FALSE(p.trajectory);
    ASSERT_EQ(p.candidates.size(), 2u);
    for (auto& r : p.candidates)
        EXPECT_EQ(r.reason, "collision");
}
TEST(ArmPlanner, SingularityIsRejectedAndAlternativeBranchCanSurvive) {
    const auto p = plan();
    ASSERT_TRUE(p.trajectory);
    bool singular = false, valid = false;
    for (const auto& r : p.candidates) {
        singular |= r.reason == "near_singularity";
        valid |= r.reason == "ok";
    }
    EXPECT_TRUE(singular);
    EXPECT_TRUE(valid);
    auto g = goal({.3, .2, .2});
    EXPECT_FALSE(
        ArmMotionPlanner(config(), free_space).plan(start, Eigen::Vector3d::Zero(), g).trajectory);
}
TEST(ArmPlanner, LimitsAndResourceBudgetAreEnforced) {
    auto c = config();
    c.limits->joint_max_rad[0] = .2;
    EXPECT_FALSE(ArmMotionPlanner(c, free_space)
                     .plan(start, Eigen::Vector3d::Zero(), goal({.3, .8, -.5}))
                     .trajectory);
    c = config();
    c.max_path_step_rad = 1e-12;
    auto p =
        ArmMotionPlanner(c, free_space).plan(start, Eigen::Vector3d::Zero(), goal({.3, .8, -.5}));
    EXPECT_FALSE(p.trajectory);
    ASSERT_FALSE(p.candidates.empty());
    EXPECT_EQ(p.candidates[0].reason, "sampling_budget_exceeded");
}
TEST(ArmTrajectory, EndpointAndAnalyticSpeedAccelerationBounds) {
    auto c = config();
    MotorAngleCalibration coupled;
    coupled.motor_to_joint << 1, 0, 0, 0, 1, 0, 0, 1, 1;
    c.mapping = MotorAngleMapping(coupled);
    c.motor_speed_rad_s = Eigen::Vector3d::Constant(.15);
    auto p =
        ArmMotionPlanner(c, free_space).plan(start, Eigen::Vector3d::Zero(), goal({.3, .8, -.5}));
    ASSERT_TRUE(p.trajectory);
    auto& t = *p.trajectory;
    for (int i = 0; i <= 1000; ++i) {
        auto s = t.at(t.duration_s() * i / 1000.);
        EXPECT_TRUE(
            (s.joint_velocity.cwiseAbs().array() <= c.joint_speed_rad_s.array() + 1e-10).all());
        EXPECT_TRUE(
            (s.joint_acceleration.cwiseAbs().array() <= c.joint_accel_rad_s2.array() + 1e-10)
                .all());
        EXPECT_TRUE(
            (s.motor_velocity.cwiseAbs().array() <= c.motor_speed_rad_s.array() + 1e-10).all());
        EXPECT_TRUE(
            (s.motor_acceleration.cwiseAbs().array() <= c.motor_accel_rad_s2.array() + 1e-10)
                .all());
    }
    EXPECT_NEAR(t.at(0).joints.upper_elevation, start.upper_elevation, 1e-12);
    EXPECT_LT((t.at(t.duration_s()).tcp_m - goal({.3, .8, -.5}).base_tcp.translation()).norm(),
              1e-8);
    EXPECT_LT(t.at(t.duration_s()).joint_velocity.norm(), 1e-12);
    EXPECT_THROW(t.at(-1), std::invalid_argument);
}
TEST(ArmFeedback, SettlesAndDoesNotCallCommandSuccessArrival) {
    auto p = plan();
    ASSERT_TRUE(p.trajectory);
    const auto& t = *p.trajectory;
    ArmExecutionMonitor m(t, policy(t.duration_s()), free_space, 0);
    ExecutionReport r;
    const double dt = .01;
    for (double now = 0; now < t.duration_s() + .2; now += dt) {
        r = m.update(feedback(t, now), now);
        ASSERT_NE(r.status, ExecutionStatus::Failed) << r.reason;
        if (now < t.duration_s())
            EXPECT_NE(r.status, ExecutionStatus::Arrived);
    }
    EXPECT_EQ(r.status, ExecutionStatus::Arrived);
}
TEST(ArmFeedback, RejectsStaleFaultTimeRollbackAndTrackingError) {
    auto p = plan();
    ASSERT_TRUE(p.trajectory);
    const auto t = *p.trajectory;
    auto make = [&] { return ArmExecutionMonitor(t, policy(t.duration_s()), free_space, 0); };
    auto m = make();
    EXPECT_EQ(m.update(feedback(t, 0), .2).status, ExecutionStatus::Failed);
    EXPECT_EQ(m.update(feedback(t, .21), .21).status, ExecutionStatus::Failed);
    m = make();
    auto f = feedback(t, 0);
    f.fault = true;
    EXPECT_EQ(m.update(f, 0).status, ExecutionStatus::Failed);
    m = make();
    m.update(feedback(t, .1), .1);
    EXPECT_EQ(m.update(feedback(t, .09), .09).status, ExecutionStatus::Failed);
    m = make();
    f = feedback(t, 0);
    f.motors.base_motor += .4;
    EXPECT_EQ(m.update(f, 0).reason, "tracking_error_exceeded");
}

TEST(ArmFeedback, RejectsOverspeedGapsTimeoutAndChangedEnvironment) {
    auto p = plan();
    ASSERT_TRUE(p.trajectory);
    const auto t = *p.trajectory;
    auto make = [&] { return ArmExecutionMonitor(t, policy(t.duration_s()), free_space, 0); };
    auto m = make();
    auto f = feedback(t, 0);
    f.motor_velocity[0] = 2;
    EXPECT_EQ(m.update(f, 0).reason, "feedback_overspeed");
    m = make();
    m.update(feedback(t, 0), 0);
    EXPECT_EQ(m.update(feedback(t, .1), .1).reason, "feedback_gap_exceeded");
    m = make();
    const double late = t.duration_s() + 3;
    EXPECT_EQ(m.update(feedback(t, late), late).reason, "execution_timeout");
    ArmExecutionMonitor blocked(
        t, policy(t.duration_s()), [](auto&, auto&) { return CollisionVerdict::Collision; }, 0);
    EXPECT_EQ(blocked.update(feedback(t, 0), 0).reason, "feedback_collision");
}
