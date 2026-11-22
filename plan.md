# 开发计划

## 2026-11-21 第三步：末段TCP直线

- 原末段关节插值在日志中计划/实际最大离线偏差约13.4mm。新增直线回归先在旧实现失败（最小用例5.77mm），替换后通过；不是靠调伺服增益掩盖弯曲。
- 新增plan_cartesian：固定朝向、同分支逐点IK、沿线FK/限位/J/碰撞采样；不允许失败退化成关节弯线。前两段仍关节空间，detour末段启用直线。
- 通过J反算路径一阶导数、Jdot曲率项求二阶导数，仿射映射到电机速度/加速度；按采样峰值、TCP限速限加速度及1.1数值余量设置时长。
- 实际TCP对直线偏差另有显式阈值，仿真1mm越界失败锁存；CSV记录segment/cartesian、计划TCP、计划与实际线偏差。不拿终点到位替代路径质量。
- 验证：运动相关18条GTest、后端CTest18/18程序、MuJoCo8条集成通过。有限差分独立核对关节速度、加速度与J贡献。
  同一相机绕行场景计划线偏差约1.1e-16m、实际最大约0.017mm，11.756s到位；仅理想仿真，不是实物精度。未commit/push。

## 2026-11-21 第二步：绕行与相机

- RoutePlan先试直接路，再尝试显式缩回/抬高经过点序列；每段复用IK/FK/限位/J/碰撞采样，全部通过才执行，段间由持续实测到位切换。
- 已复现直接路径碰撞拒绝→选择三段绕行→到位。有限候选搜索不是RRT、不保证找全路线；尚无TCP直线最后接近与抓取。
- 文本桥新增相机位姿输入，使用arm/observation同源时间配对和基座变换，再组合明确的标记板到TCP任务偏置；不消费目标真值。
- 模拟末端相机实际RGB→前端四色圆标中心检测/PnP→后端目标→绕行→反馈。当前只在运动前识别一次固定目标，不是持续视觉伺服。
- 工具水平任务把小量姿态噪声约束到可实现yaw；先检查旋转偏差及侧偏阈值，超限拒绝，不强行忽略不可实现姿态。阈值全部为仿真参数。
- 验证：运动相关14条GTest，后端18/18测试程序；前端9/9测试程序，含3条圆标测试；MuJoCo集成8条通过。
  相机三段绕行7.754s到位，终点对合成真值误差约0.117mm；不能宣称实物精度。无commit/push。

## 2026-11-21 点到点仿真链路

- 新增arm/motion_pipeline：TCP偏置→水平机构解析IK→连续角候选→关节路径采样/FK/静态限位/位置J/外部碰撞→运动量排序→五次时间缩放→反馈监测。
- C++仿真桥供MuJoCo逐步请求轨迹与回报实际角度/速度/时间；不把目标写成qpos实际值。
- 未知碰撞/无检查器拒绝；采样检查有分辨率与资源上限，当前不支持连续碰撞证明、自动绕障或TCP直线接近。
- 仿真模型三主动轴+被动水平约束；碰撞代理不含完整自碰撞/实物外形，所有合成参数不可作为实物配置。
- 下一步：复核模型和日志，逐段读代码；实测参数到位后完善碰撞模型，再接相机。不得因仿真到位打开实物运动。
- 验证：新增运动相关12条GTest、后端CTest 18/18测试程序通过（先source ROS Humble）；MuJoCo集成5条通过。
  默认轨迹1.201s、到位确认1.464s/732步，模拟终点误差1.38e-6m；仅为合成模型数值结果。窗口短测退出0，未人工验收画面。
  black/ruff、clang-format及diff --check通过；未commit/push。仿真脚本和日志不混入后端业务仓。

## 2026-11-20 静态目标检查与闭环边界

- 目标检查顺序固定为配置与有限性→几何域上下界→映射→电机域上下界；失败明确报告域/轴且不产出电机目标，不夹紧。限位须外部显式提供，无实机默认参数。
- 只做静态目标检查，不冒充当前状态准入、时效、轨迹、碰撞、奇异或到位判断；未接通讯/电机。新增5条GTest通过，后端构建及CTest 17/17测试程序通过，clang-format及git diff --check通过；未commit/push。
- 最近三个交付：①当前/目标约束、反馈时效与到位小用例；②FK-IK回代和点到点仿真闭环；③视觉手眼与抓取状态流程。每一步分别验收，不先装大型仿真器或把JD加分项当全部前置。

## 2026-11-20 映射接口预留

- 按本人要求新增电机连续角(rad)与LevelArmAngles的双向仿射纯计算接口，支持零位、方向、比例及线性耦合；默认未配置，禁止静默用单位映射。
- 新增6条合成参数GTest，包含FK衔接、数值退化与溢出；后端构建成功、CTest 16/16测试程序通过，clang-format与git diff --check通过。未使用实物标定，未接设备，未commit/push。
- 限位尚未实现：后续目标在发送前检查几何/电机域约束，反馈用于独立监测，不能只等反馈超限再处理。实际约束与故障后的停止方式待机械电控确认。

## 2026-11-20 机械臂几何可读性与离线回归

- 将正运动学局部变量明确为大小臂水平/竖直贡献、末端伸出量/高度、底座yaw正余弦；公共接口与算法不变。
- 补充上下相反仰角、非等长杆和125组几何组合检查；仅数学回归，不是硬件限位/碰撞或运动闭环。
- 验证：加载ROS Humble环境后后端构建成功，CTest 15/15测试程序通过（125组几何组合包含在test_arm_geometry内部，不计为125个CTest）；clang-format已应用，git diff --check通过。未接电机，未commit/push。下一步再做逆解接口与失败条件评审。

## 2026-11-14 include业务分区（覆盖旧路径）

验证：加载ROS Humble环境后重新构建，CTest 15/15 测试程序通过；仓内旧根路径include扫描为0，git diff --check通过。未运行真实硬件，外部历史调用方尚未迁移。

头文件与src同步按业务分区；装甲板armor/、机械臂arm/，前端检测抽象放common/。所有仓内源码、工具与测试include同步更新。库目标、命名空间和算法不变；这是公共包含路径破坏性变更，不保留旧路径转发头，外部调用方须同步改include。未改其他工作区或历史模拟器源码，未commit/push。


## 2026-11-14 平台依赖已接通（覆盖下文历史阻断）

- 固定diesel_platform的origin/dev提交58e617fd56a71b012ccc6b94e2711591bc1741ff，工作区内构建/安装diesel_platform_core，通过显式diesel_platform_core_DIR供后端查找。
- 后端完整构建通过，CTest 15/15测试程序通过；平台USB/RoboLink筛选测试39/39通过。未执行平台全部218项测试。
- 未删除或替换生产依赖，未改平台源码，未安装系统级权限规则；无真实硬件控制。工作区根build_backend.sh记录构建顺序。
- 两仓未commit/push，平台固定提交只读使用；正式元仓依赖版本锁待本人评审。

## 2026-11-14 源码业务分区

验证：机械臂6项几何检查、观测变换GTest通过。全仓仍缺diesel_platform_core，不能宣称全仓构建通过；不改平台依赖绕过检查。

按本人要求，将装甲板实现迁入src/armor，机械臂实现归src/arm。CMake同步源文件路径，公共头文件/命名空间/算法不变。前端观测校验由inline移为src/arm/observation.cpp，测试语义不变。未commit/push，PR仍到dev。

## 2026-11-14 机械臂分支

验证：新增几何6项检查及观测变换GTest独立编译运行通过；全仓配置因未安装diesel_platform_core而阻断，未伪造依赖或宣称全仓通过。
- 从 origin/dev b97bed6 建立 feat/arm-backend，PR目标dev；未commit/push。
- 已迁入三自由度LevelArmModel，命名空间diesel_vision_backend::arm；149mm来自本人说明，轴心距待复核。
- 完整相机目标位姿和同采样相机变换输入、时效与SE(3)校验独立实现，不依赖前端/OpenCV。
- 旧四轴任务仿真未迁入正式链路；逆解、电机映射、碰撞和实机仍未实现。

## 任务列表

- [ ] 接 `diesel_vision_frontend` 的逐帧 `Observation`，实现多源目标融合与注入
- [ ] 实现友军**过滤**（进入候选集之前）—— 核心函数与 7 条测试已落地，待接入链路
- [ ] 实现弹道解算与云台伺服（硬实时环，独立线程、禁动态分配）—— 弹道暂不做 直瞄已落地 伺服环待做
- [x] **整车状态估计 EKF**（11 维，`predict` / `update` / 观测雅可比 `h_jacobian` + 仿真测试）—— 2026-10-03 落地
- [x] 单目标跟踪核心：四板位置关联、连续确认、短暂丢帧和时间跳变重置（离线测试）；尚未接入真实 Vision 数据流
- [x] 将 Tracker 的输入目标类别与 Vision 对齐为 -1/0/1/2；重装仅预留，不代表现有模型已识别
- [x] 旋转靶离线回归：板位置 + 朝向联合更新、角度 wrap 与角过程噪声；原地旋转的中心漂移测试由失败转为通过
- [x] 快转初值与关联门控离线回归：模拟器适配层放宽角速度初值方差，Tracker 的关联同时检查位置与朝向；仍待模拟器完整旋转日志和实测门限验收
- [ ] 用连续真实模型观测验证上述旋转修复；统计中心漂移、错板次数与检测缺失帧，不能把单元测试当闭环通过
- [ ] SP 双模型同帧后端对照：分开记录主关联残差、主板与次板更新导致的车心修正，再做单变量门限 A/B；当前仅增加离线诊断，尚未证明门限根因
- [x] 将 SP `Target::update` 的近三板、朝向差＋方位差选板规则移成独立纯函数并测试；已作为 `SpNearestThree` 可选策略接入 Tracker，默认仍为 Foray 距离门；尚非完整 SP 跟踪基线
- [x] 模拟器 100 帧同板离线诊断：记录角点、PnP 与关联拒绝原因；确认双模型四角系统性内缩、PnP 测距偏远，59 帧被 0.25 m 距离门拒绝。尚未修复模型输入或接实机。
- [x] 离线 v1：由整车估计按局部板号预测未来位置并计算云台相对角，接入模拟器适配层；默认提前量为 0。旧 ONNX 模型实跑 120 帧，曾进入 Tracking、仿真云台反馈角度变化且 fire=false；途中仍有 Lost，尚未做 A/B 统计
- [ ] 在同一模拟场景对比原始 PnP 直瞄、整车预测 0 ms 和不同提前量的跟随误差；确认高速换板时预测板是否仍可见，再决定切板策略
- [ ] 将同帧 PnP 姿态和 IMU 转为稳定坐标系后接入 Tracker，并显示预测四板；再验证锁定目标选择
- [ ] 定义开火门限的目标准入契约（对接 X1 安全门）
- [x] 离线适配云台系 `GimbalCommand` + 有效 IMU 四元数到 v0.2 世界系目标角控制帧，验证旋转、单位和禁用条件
- [x] 将适配帧接入平台主机链路排队入口；握手与状态回读前拒绝控制
- [ ] 完成图像与 IMU 姿态的时间对齐及真实标定外参验证，再接入装配层真实 USB 控制
- [x] （已迁出）独立实机禁能联调窗口：海康相机→双模型→可选 PnP/静止相机 Tracker；RoboLink 只握手、保活和回读 —— 调试入口归 `diesel_vision_frontend`
- [ ] 现场测量装甲板实际外轮廓尺寸与板心—车心半径，验证相机系 PnP 和静止相机 Tracker；记录真实检测/漏检/换板日志
- [ ] 实机禁能联调核验 v0.2 世界系 rad 目标角、状态回读与控制权裁决
- [ ] 连靶车后另起独立 Foray 视觉调试软件，参考现有 MIT 模拟器的接口/可视化思路；先完成当前跟踪与实机验证，不在本轮启动新项目

## 当前 Agent State

- [x] 统一C++接口AimPipeline与独立foray_auto_aim_runtime已实现：现有Vision→候选手眼→主机时刻姿态插值→世界观测→Tracker/EKF→选板/预测→直瞄→C++ USB线程；调试台只启动C++进程并显示，不再调用Python运行桥/worker。
- [ ] 新C++入口实机运动复测：当前MVS枚举不到海康相机（USB列表亦无该相机），不能完成当前取图闭环验收；MCU串口仍存在。不自动发送运动。
- [ ] 整车底盘平移场景需接入云台世界原点/里程计，目前固定原点假设仅适合当前台架；精确对时、手眼精度、半径漂移与换板稳定性仍未验收。

- [x] 2026-11-11：独立world_tracker_worker已串联世界观测→Tracker/EKF→选板/预测→直瞄；外部工具独占串口，20秒实机短测已完成。几何漂移和精确对时未验收，不能宣称完整比赛后端通过。
- [x] 世界系WorldAimSolver替换边界与直瞄实现，弹道由另一队员实现；当前不接弹道或发射。
- [ ] 后续诊断实测半径漂移、候选外参和板朝向输入；完成同场景A/B与运动对时验收。加速度观测器与电控前馈没有在本轮实现。
- [x] 2026-11-14：视觉前后端分家第一刀（I2）：本仓收敛为后端，只消费前端逐帧 `Observation`（相机系位置/法向/板号）；
  删除 `tools/`（相机调试归前端）、`camera_diagnostic_adapter` 与 `vision_adapter`，去掉 OpenCV 依赖；
  `AimPipeline::process_observations` 不再接收图像或检测器；`Target` 只保留 EKF，NIS 野值门限由 Tracker 关联层承担。
  本地 11 个测试程序 82 条全绿；CI 待 PR 触发。
- [x] 2026-11-13：时间门限三档统一 + 姿态时间偏移 + 图案模型边长可配；实机闭环 `control_seq/rx` 只差 1，稳态误差 1.4 mrad，`rejected_backward=0`。103 条测试全绿（diesel_vision 20 + 本仓 83）。后端收敛后本仓为 82 条，前端 20 条待 I3 收敛后合并统计。
- [ ] 曝光绝对时刻标定：屏幕闪烁(PRBS)+固定曝光+自动开始采集+互相关分析链路已就位；两次采集均因相机未正对屏幕作废（最亮帧为暗墙面）。参数 `--exposure-offset-ms` 已接入取帧处，默认 0 待填。

- [x] 2026-11-11：只读入口增加 `--target-feed-json`，复用双模型/PnP 导出逐帧观测（含空帧），供人工静止点动验证；未接入运动 Tracker 或自动发射。

TRACKED_AIM_SIM_V1_RUNNABLE：整车预测已进入仿真控制输出，旧模型 120 帧短跑出现 Tracking 与 Lost；尚无 A/B 指标、图像/IMU 时间对齐或真实电机控制。

## Before Snapshot

commit hash:   3e6f90f
branch:        feat/detector
modified files: 保留既有 detector/vision_adapter 等未提交改动；本轮新增 control_frame_adapter 与测试，并更新构建依赖及文档
risk level:    L1

## 模糊点与待确认项

- 本仓接口尚未冻结，任务清单为**方向性**的，落地顺序以 `foray_docs/algorithm_structure.md` §9 演进阶段为准
- 依赖的自研仓尚未建齐，跨仓任务需等对方 `README.md` 明确接口后再启动
- `package.xml` 的 `license` 暂为 `TODO`：组织文档未规定私有仓许可证，待确认后补
- 敌我标识的最终来源待定：接口冻结前本模块用自己的 `TargetCandidate`；冻结后需补一层「上游消息 → TargetCandidate」转换

## Vector Backend Status

Backend: Markdown
Status:  ready
Environment: 仓库内 Markdown 文档（无外部向量后端）
Index: 本仓 `README.md` / `tree.md` / `decision.md`
Initialization: 2026-02-19
Commit: （首次提交）

## Acceptance Criteria

- [x] 本仓能独立 `colcon build`（本地：1 package finished）
- [ ] CI 绿灯（待 PR 触发）
- [x] 每个新增模块带独立可执行测试（友军过滤 7 条 · 云台角 5 条 · 整车 EKF 1 条，共 13 条全绿）

## 2026-11-14 I4 仓名与包名迁移

- GitHub 仓 `diesel_auto_aim` → `diesel_vision_backend`，本地目录与 `origin` 同步；PR #8 先合入 `dev`。
- 包名 / CMake `project()` / `include/diesel_vision_backend/` / `namespace diesel_vision_backend` / guard `DIESEL_VISION_BACKEND_*` 全量改名。
- 本地验证：`cmake -DBUILD_TESTING=ON` 构建通过，11 个测试程序 83 条全绿；后端仍只链 `Eigen3::Eigen` + `diesel_platform::link`。
- 下一步：I5 前端多目标抽象、I6 interfaces 视觉消息、I7 Fusion/WorldSnapshot。

## 2026-11-14 I7 融合层与 WorldSnapshot

- 新增 `fusion.hpp/.cpp`：`fuse_observations()` 在几何变换前挡掉过期（>100 ms）、非装甲板类别、
  非法板号/几何的观测；同板号且距离 <0.15 m 判为同一块板，**自车相机优先**，不平均位置。
- 新增 `world_snapshot.hpp/.cpp`：`build_world_snapshot()` 输出 0/1 条 `SnapshotTrack`（世界系位置/速度/置信度/状态），
  `track_confidence()` 让 `Lost=0`、`Detecting≤0.5`、`Tracking` 随丢帧衰减、`TempLost=0.2`。
- `AimPipelineResult` 增加 `snapshot`、`fused_count`、`fused_dropped`、`fused_merged`；
  `AimPipelineConfig` 增加 `FusionConfig fusion`。
- 未打戳观测（`stamp_s <= 0`）按本帧戳处理，显式打戳的仍按自身时间判过期。
- 测试：新增 `test_fusion`（8 条）与 `test_world_snapshot`（4 条），`test_aim_pipeline` 补 3 条
  （多源合并、过期观测被融合层挡掉、快照只在确认后暴露轨迹）；全仓 98 条 + 前端 28 条 = 126 条全绿。
# 2026-11-22 顶抓路线

- 已实现后端强制准备→水平对准→竖直下降；朝下工具与MuJoCo TCP一致。
- 固定安全高度仅为合成场景配置；不具备任意障碍绕行完备性，真实夹爪尚未接入。
