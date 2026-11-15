# 目录结构说明

```text
.
├── include/diesel_vision_backend/
│   ├── armor/   自瞄观测、融合、跟踪、EKF、预测、云台指令接口
│   └── arm/     三自由度几何与相机到基座目标转换
├── src/
│   ├── armor/   装甲板业务实现（当前融合/快照有装甲板假设）
│   └── arm/     level_arm.cpp、observation.cpp
├── test/        各模块独立无硬件测试
├── CMakeLists.txt / package.xml
├── README.md / plan.md / decision.md
└── AGENTS.md / .diesel-layer / .github/
```

## 包含路径与边界

2026-11-14：include与src同时按业务划分。公共包含路径是破坏性变更；仓内源码、测试和工具已同步，外部调用方需要同步修改。命名空间保持`diesel_vision_backend`（机械臂后端使用其`::arm`子命名空间），库目标名称不变。不保留根目录旧路径转发头。

后端不依赖OpenCV/推理运行时。机械臂只实现简化几何与时间/坐标校验，未接真实电机/夹爪。装甲板时效策略带其现有实测参数，因此暂归armor，不强行共用给机械臂。

## 记录约束

不记录build/install/log/.git或缓存产物。
