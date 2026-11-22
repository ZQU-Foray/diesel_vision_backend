# 目录结构说明

2026-11-21：motion_pipeline增加固定TCP朝向的直线轨迹模式及J/曲率导数；route_plan可要求末段直线。
motor_angle_mapping增加逆速度/加速度线性映射；不复用位置零位。无新增依赖或真实设备入口。

2026-11-21：`arm/route_plan.hpp`及`src/arm/route_plan.cpp`组合有限经过点候选，每段用motion_pipeline验证。
仿真桥处理分段到位转换、相机测量适配；不反向链接前端或OpenCV。

2026-11-21：`arm/motion_pipeline.hpp`及`src/arm/motion_pipeline.cpp`集中实现TCP IK、位置J、
采样路径筛选、五次时间缩放和反馈监测；`test/test_arm_motion_pipeline.cpp`覆盖正常与拒绝路径。
`src/arm/motion_sim_bridge.cpp`为文本进程适配入口，连接工作区MuJoCo，不在演示脚本重复IK。
碰撞以外部检查器注入；缺失/未知/异常即拒绝。仿真入口只有合成杆胶囊/球障碍/地面代理，不代表完整自碰撞或实机安全。
`motor_angle_mapping`新增角速度线性映射，避免把零位混入速度计算。

```text
.
├── include/diesel_vision_backend/
│   ├── armor/   自瞄观测、融合、跟踪、EKF、预测、云台指令接口
│   └── arm/     三自由度几何与相机到基座目标转换
├── src/
│   ├── armor/   装甲板业务实现（当前融合/快照有装甲板假设）
│   └── arm/     level_arm.cpp、motor_angle_mapping.cpp、target_angle_check.cpp、observation.cpp
├── test/        各模块独立无硬件测试
├── CMakeLists.txt / package.xml
├── README.md / plan.md / decision.md
└── AGENTS.md / .diesel-layer / .github/
```

## 包含路径与边界

2026-11-14：include与src同时按业务划分。公共包含路径是破坏性变更；仓内源码、测试和工具已同步，外部调用方需要同步修改。命名空间保持`diesel_vision_backend`（机械臂后端使用其`::arm`子命名空间），库目标名称不变。不保留根目录旧路径转发头。

后端不依赖OpenCV/推理运行时。机械臂只实现简化几何与时间/坐标校验，未接真实电机/夹爪。装甲板时效策略带其现有实测参数，因此暂归armor，不强行共用给机械臂。

2026-11-20：`arm/motor_angle_mapping.hpp`与对应cpp提供可配置、默认未就绪的电机角/几何角双向仿射转换；`test_motor_angle_mapping.cpp`验证参数、方向/比例/零位、耦合、连续角和异常输入。只有数学草案，不含实际标定或安全限位。

## 记录约束

2026-11-20：`arm/target_angle_check.hpp`与对应cpp负责单个目标的几何/电机域区间检查，`test_target_angle_check.cpp`验证拒绝原因、边界和耦合映射；不是完整控制安全门。

2026-11-20：`src/arm/level_arm.cpp`明确四项杆投影；`test/test_arm_geometry.cpp`补上下相反仰角、非等长杆及125组姿态几何回归。未新增依赖或运行入口。

不记录build/install/log/.git或缓存产物。
顶抓路线：`arm/route_plan`新增`plan_top_down_route`，准备段后强制水平与竖直两段TCP直线。
