# 来源与许可标注

- 上游：<https://github.com/agilexrobotics/agx_arm_sim>
- 冻结提交：`f8cd8b147c75d59e14f90fb0646770eefa268ed0`
- 资产路径：`mujoco/agilex_arm/agilex_nero/`
- 比较条件：Issue #7已保存kp80的`model_B`；桌面/方块任务场景与控制器配置保持该基线。
- 派生变更：仅替换link5/法兰的碰撞几何，保留官方原始网格，移除诊断用link5/link7整对排除。`assets/collision_candidate/`为本轮从原始三角面和既有局部特征块派生的凸代理。
- 独立加载修正：将含方块自由度的任务home关键帧从机器人文件移到场景文件；移动前后完整场景的MuJoCo编译二进制一致，机器人文件可以单独加载。没有更改关键帧数值或控制接口。
- 未使用经销商STEP、厂家未确认CAD或其它自建NERO模型。

冻结提交的上游README声明“MIT License”，其原始文本保存在`UPSTREAM_README.md`。该Git树未发现独立LICENSE/COPYING文件或逐网格版权声明；此处不编造版权持有人或补写授权。正式对外Release前应补齐应随包附带的完整许可通知。当前仅为本地技术审查候选。
