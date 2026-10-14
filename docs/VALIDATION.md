# 验证结果与演示

## 结论

**当前 76 块手部代理的来源模型已完成阶段性资产接入与动态接触验证。** 验证包含源 URDF 机械字段、18D／13D 控制契约、原生 LIBERO 环境，以及白碗闭合、抬升和持续对向持握。原始阶段核对记录明确标记 `asset_contract_and_runtime_verified: true`。[验收记录][audit]

验证范围是记录中的碰撞近似、位置控制参数和安装假设。本页区分来源模型已经执行的测试与资产包附带的静态入口，不把“未附完整动态测试工程”写成“没有做过动力学测试”。

## 动态接触结果

环境为原生 LIBERO90 `KITCHEN_SCENE9_put_the_white_bowl_on_top_of_the_cabinet`；选用原尺寸白碗，可视最大跨度约 84.9 mm。执行组合为 NERO＋左 DH116、76 块手部代理、8 块法兰代理、原生 JPC 与 MuJoCo position；MuJoCo 3.2.3，RK4／2 ms，seed `20261007`。[场景与控制][frozen]

| 检查项 | 记录结果 |
| --- | --- |
| 动作执行 | 230 个 13D 脚本命令，5750 个物理步，11.5 s |
| 运行状态 | `DIAGNOSTIC_FINISHED`，无停止原因；报告无数值警告 |
| 接触与操作 | 已观察到闭合、抬升；最后 1.5 s 的 750/750 步保持对向接触 |
| 物体支撑 | 抬升／保持代表帧核对为手部支撑 |
| Mimic 峰值偏差 | 约 0.001001 rad |
| 最大手—碗代理交叠 | 约 0.580 mm |
| 原生完整任务 | 未执行放到柜子阶段，`native_task_result: false` |

以上来自[动态结果][dynamic]、[支撑与源表面核对][surface]和[阶段核对][audit]。物理步进由实际 `env.step`／MuJoCo 执行；视频是记录状态的只读渲染。它们是模型接入与接触可行性证据，不是学习策略评测或正式示教标签。

## 碰撞近似对照

68→76 块只细分两个主要接触指尖，各由 8→12 块；同一白碗、seed、230 步命令和控制参数用于一次实际动态对照，没有重新规划或通过改物体质量、摩擦获得结果。[对照说明][comparison]

| 指标 | 68 块 | 76 块 |
| --- | ---: | ---: |
| 拇指误占距离 P95 | 1.188 mm | 1.011 mm |
| 食指误占距离 P95 | 1.239 mm | 1.063 mm |
| 动态最大手—碗代理交叠 | 约 0.695 mm | 约 0.580 mm |
| 白碗抬升 | 已观察到 | 已观察到 |

几何指标采用各源 link 包围盒内相同的 1024 个 Sobol 样本，度量处于代理内、源表面外的误占点到源面的距离；不是全表面误差≤1 mm 的保证。动态数字来自指定单次运行，不据此推算任务成功率。

## 视频

| 演示 | 说明 |
| --- | --- |
| [76 块：白碗闭合与抬升][video76] | 当前来源版本，侧上方视角 |
| [68 块：白碗闭合与抬升][video68] | 同条件代理对照；该版本也完成了抬升 |
| [早期黑碗失败：单侧接触，未抓住][failurevideo] | 不同物体／布局条件下的失败案例，不能作为仅改变代理的消融对照 |

早期失败视频对应[该次接触诊断][failure]。 失败记录显示短时食指接触、未形成对向夹持，不能笼统归因于代理未改进或碗太大。两段白碗视频也不能标注为“失败→成功”。

以上为已有归档视频入口，不是本次新仿真或重新上传的媒体。视频和详细证据位于原项目的权限受限仓库，需要该仓库访问权限。

## 与交付资产的对应

本包 [SOURCE_MANIFEST.json](../SOURCE_MANIFEST.json) 指向已执行并冻结的 `verified_white_bowl_scene_76/scene.xml`；源场景 SHA256 与[冻结清单][frozenhash]一致：

```text
4cbf4eb09ae18cf780d27d5e3a4956b0d01e66066b3dfe69f4752daa8ed5427d
```

冻结记录核对了 27 类物理／接触字段及实际 reset；交付提取另对同名机器人实体的结构、物理参数、驱动和参考姿态进行静态核对。两者构成来源关联，不表示独立资产包已经重新跑过完整动态场景。Manifest 中提取阶段的 `physics_steps: 0` 不否定来源实验的 5750 个物理步。

本仓库的 `examples/view_static.py` 与静态测试只负责加载、显示和交付检查。已有动态执行适配及原始轨迹留在实验工程，不在这里复制完整开发过程。

## 验证边界

已完成的是所述条件下的阶段性仿真与接口可行性验证。没有新版学习策略、完整柜子放置任务或多 seed 成功率；真实驱动力矩／速度、安装、相机和接触材料尚未标定。有限姿态与单次抓持不构成全工作空间或实机一致性保证。这些限制不撤销已经完成的动态接触验证。

原始证据固定到实验仓库提交 `0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a`；[阶段结论][conclusion]给出完整适用范围。本页为历史结果的交付说明，本次文档修正没有重新执行物理实验。

[audit]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/asset-integration-completion-audit.json
[frozen]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/runtime/verified_white_bowl_scene_76/README.md
[dynamic]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/moderate-contact-tip-proxy/native-command-comparison/dynamic-results.json
[surface]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/moderate-contact-tip-proxy/native-command-comparison/physics-and-source-surface-review.json
[comparison]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/moderate-contact-tip-proxy/README.md
[video76]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/moderate-contact-tip-proxy/native-command-comparison/closure-side.mp4
[video68]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/smaller-bowl-selection/white-bowl-closure-diagnostic-prepared/closure-side.mp4
[failurevideo]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/settled-native-attempt/diagnostic.mp4
[failure]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/README.md
[frozenhash]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/runtime/verified_white_bowl_scene_76/SHA256SUMS
[conclusion]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/%E8%B5%84%E4%BA%A7%E4%B8%8E%E8%AE%AD%E7%BB%83%E6%8E%A5%E5%8F%A3%E9%AA%8C%E8%AF%81%E7%BB%93%E8%AE%BA.md
