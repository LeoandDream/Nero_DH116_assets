# 验证条件与适用范围

## 验证范围

当前模型已在参考配置下完成源机械字段核对、18D／13D 接口检查和 LIBERO 白碗动态接触验证。验证对象为 NERO＋DH116 左手、76 块手部代理及 8 块法兰代理。[验证摘要][audit]

| 层面 | 覆盖范围 |
| --- | --- |
| 结构与转换 | 关节轴、范围、质量、惯性、线性 Mimic、驱动与参考姿态的字段核对；有限姿态和几何检查 |
| 控制与状态 | 18D 关节位置、13D 独立位置动作及原生位置控制适配 |
| 动态接触 | 白碗闭合、抬升、保持；接触支撑、Mimic 残差、代理交叠和数值警告检查 |
| 仓库自带检查 | 文件 SHA256；静态查看器显示过滤及测试所覆盖物理字段的不变性 |

当前独立复现入口和实际验收记录见[参考测试](../examples/libero_reference/README.md)与[验收证据](../examples/libero_reference/evidence/RESULTS.md)。公开固定依赖与必要夹具随交付提供，运行无需访问历史实验仓库。

## 白碗抓取工况

参考环境为 MuJoCo 3.2.3、robosuite 1.4.1 和原生 LIBERO90 厨房桌面场景。白碗采用原尺寸，visual 最大跨度约 84.9 mm；控制采用原生 JPC 绝对关节位置入口与 MuJoCo position 伺服，RK4／2 ms，seed `20261007`。

| 指标 | 结果 |
| --- | --- |
| 脚本动作 | 230 个 13D 命令 |
| 仿真时长 | 11.5 s，5750 个物理步 |
| 操作表现 | 完成闭合与抬升；末段 1.5 s 的 750/750 步持续对向接触 |
| 相对 Reset 最终增高 | 33.362942 mm，Reset z=0.97 m |
| 相对沉降最终抬升 | 103.952232 mm，零起算行249 z=0.8994107101064139 m |
| 严格抓取成功声明 | false |
| 支撑检查 | 抬升／保持代表帧为手部支撑 |
| Mimic 峰值偏差 | 约 0.001001 rad |
| 最大手—碗代理交叠 | 约 0.580 mm |
| 数值状态 | 执行结束，无停止原因；无数值警告 |

结果来自一次固定配置的脚本抓取测试。工况覆盖抓取和持握；原场景的柜子放置阶段未执行，完整任务判定为 false。代理交叠是求解器几何指标，测试结果不包含真实材料穿透深度或学习策略成功率。[详细测量][dynamic]

[查看白碗闭合、抬升与持握视频][demo]

## 资产与复现信息

| 项目 | 标识 |
| --- | --- |
| 机器人模型入口 | [`mjcf/nero_dh116.xml`](../mjcf/nero_dh116.xml) |
| 接口定义 | [robot-contract.json](robot-contract.json) |
| 网格和源场景映射 | [SOURCE_MANIFEST.json](../SOURCE_MANIFEST.json) |
| 参考场景 | [verified_white_bowl_scene_76][frozen] |
| 测试材料版本 | `0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a` |

组合 MJCF 提取自上述参考场景的机器人部分；对应源场景 SHA256 为：

```text
4cbf4eb09ae18cf780d27d5e3a4956b0d01e66066b3dfe69f4752daa8ed5427d
```

本仓库可独立加载模型、运行静态检查与固定白碗动态参考测试。场景来自公开固定 LIBERO，控制适配、动作输入、Reset/物理验收夹具和压缩基准轨迹均随仓库交付。原始探索记录的权限受限链接仅供可选溯源。

## 尚未覆盖

未提供全工作空间接触保证、多 seed 任务成功率或完整柜子放置任务成绩。真实驱动力矩／速度、接触材料、安装和相机外参尚未标定。用于其他物体、精细插接、力控制或实机迁移时，需要针对相应接触尺度和控制条件补充验证。

[audit]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/asset-integration-completion-audit.json
[dynamic]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/moderate-contact-tip-proxy/native-command-comparison/dynamic-results.json
[demo]: ../examples/libero_reference/evidence/closure-side.mp4
[frozen]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/runtime/verified_white_bowl_scene_76/README.md
