# Issue #5：B + functional_v1 单次功能性抓持

**实际抓住并抬升了白碗。6500/6500步完成，退出码0；连续有效持握1.5秒，完整hold已完成。最终相对250步沉降基准抬升100.494mm。没有真正的硬停止。原生柜顶任务仍为false。**

授权：[6346434609](https://github.com/LeoandDream/Nero_DH116_assets/issues/5#issuecomment-6346434609)。基线`1cec8e94930d19223d94c11fe4dbf5c51b92024f`；旧B的245步早停、原有FAIL及高保真未决状态全部保留。本次只执行一次完整原生Reset开始的实验，未重跑旧B、A或左手，没有续跑、烟测episode、自动调参或重试。

## 实际改变及对照

新显式入口[`close149_functional.py`](../../../right_hand/close149_functional.py)要求`--protocol functional_v1`，旧入口不变。每步调用原ResearchPolicy/ContactTracker恰好一次，仅将`OUTSIDE_REGISTERED_PATH_ENVELOPE`、`ACTIVE_HOLD_TRACKING`从实际停止名单移至诊断；原判断、首次触发、逐步幅度、接触窗口和真正硬失败另行保留。没有改有效抓持谓词，也没有扩大0.02rad门槛值。

B的34属性、81手/306腕、厂家机械定义、摩擦/solver/线性Mimic、Reset、20Hz控制/2ms物理步/RK4、全部260条close149动作及lift首条重复起点不变。运行前无同类输出/进程，原子创建新尝试目录，见[execution.json](raw/execution.json)。源/资源/动作及运行前后SHA保存在原格式证据中；原B模型生成与编译核验完整复用。

原始结果继承的`only_model_change=34 frozen hand drive attributes`描述B相对原A源模型的生成差异；本轮相对旧B没有新增驱动或模型变化，真实编译快照已精确核对。

真实Reset的状态、控制器、场景、原生接触与warmstart/力缓存、formal/actual编译快照均与旧B精确一致。前245步全部35项非停止元数据及完整RK4接触缓存**逐步精确一致，无容差**；第245步只告警并继续。运行时每步比对后才允许下一步积分；不存在从245状态续跑或额外物理差异。[Reset比对](raw/functional-reset-comparison.json)、[诊断与前缀报告](raw/functional-diagnostics.json)、[独立抓持复核](grasp-review.json)可复查。

| 观测 | 旧B | 本次functional_v1 |
| --- | --- | --- |
| 实际步数 / 阶段 | 245 / settle_preshape早停 | 6500 / 全部阶段完成 |
| 路径包络首次触发 | 245，硬停 | 245，诊断；累计5108步 |
| hold主动目标误差 | 未测试 | 首5751，750步诊断，最大0.047415rad |
| 首次真正硬失败 | 路径域保护 | 无 |
| 限定白碗抓持 | UNKNOWN_EARLY_STOP | PASS_FIXED_WHITE_BOWL |
| 原生柜顶任务 | false | false |

这是研究执行口径调整后的功能性证据，不是物理模型或控制配置被改进的证据。越过旧包络后，不沿用旧有限姿态扫描声明几何安全或全域高保真。旧规则在本次轨迹上的影子判定不等于历史旧B实际执行过245步后的状态。

## 接触、离桌与持握

| 事件 | 首次物理步 | 原始记录支持 |
| --- | ---: | --- |
| 手—碗正力接触 | 4462（close） | finger13局部代理与白碗 |
| 拇食正力对向接触 | 4637（close） | 拇指/食指接触径向XY点积<0 |
| 真实离桌 | 4794（lift） | 已在62步落桌后，环境正力支撑消失，至6500持续无支撑 |
| 抬升≥20mm、无支撑且对向 | 5101（lift） | 高度相对250步post-step沉降位置 |
| 完整有效hold | 5751–6500 | 750/750步均满足原谓词，连续1.5s |

250步沉降Z为0.899410710103m，最终Z为0.999904316344m。最终相对沉降抬升**100.494mm**，最大100.497mm；相对Reset仅**29.904mm**，两种基准不混用。hold期间最低抬升86.783mm，所有750步均无任何碗—非手正力支撑，并持续存在正力拇食对向接触。原通过时长为连续≥1.0s，计划hold的完整1.5s另行确认通过。

这些支撑/对向判定由原始contacts独立重算，与state的全部6500步布尔值一致；没有仅依赖`grasp_result`或“跑满”判断成功。高度使用post-step qpos；接触/力/体位姿使用原RK4末内部阶段缓存，两者未被声称精确同一时刻，也未为观察追加forward。

## 保留的硬保护与适用范围

无nonfinite、MuJoCo数值warning/时间异常、Mimic、机械限位、臂跟踪或腕部意外外物接触停止。最大臂误差0.089029rad、Mimic残差0.002109rad、关节限位超出0.000674rad，均未触发原门槛。最大合资格接触交叠0.485276mm；旧2mm通道本次未触发，V1接触告警/停止事件均为0。逐步原V1 groups/warnings/contact_events与冻结策略独立重算完全一致，跨阶段没有清窗口，见[保护复核](protection-review.json)。

**功能性限定抓持PASS；functional_v1数值/硬保护PASS；旧协议仍会在245步停止；原生柜顶任务false。** 当前动作日程只有接近、闭合、抬升、持握，没有交付柜顶放置阶段。本次不是官方LIBERO成绩、成功率统计、整包高保真验收或正式左右手发布。

## 证据与交付

[统一episode](episode/)保存6500条state/action/ctrl、阶段、结果、相机索引与原始来源；[agentview视频](episode/videos/agentview.mp4)、[手腕视频](episode/videos/robot0_eye_in_hand.mp4)各260帧/20fps。只复用真实运行图像，未补跑或用状态渲染伪装观察。媒体只存一份，逐原文件SHA映射见[artifact-source.json](raw/artifact-source.json)。统一导出器未改，仅在本次新manifest中显式绑定真实B模型与functional_v1口径。

[raw](raw/)保存完整状态、接触/力、RK4缓存、实际执行源码/diff、模型副本、初始化、依赖与执行记录；大JSON无损gzip，原字节SHA保留。显式episode积分6500步、显式Reset1；编译器内部自检与不推进时间的API刷新单独记录，复用的A参考仅编译、零A episode步。所有后处理、独立复算和视频检查均为零新动力学。

[tests.log](tests.log)：7项functional、8项旧B、8项close149离线测试通过。[verification.json](verification.json)记录旧文件、正式资产、main和用户README保护，完整SHA见[SHA256SUMS](SHA256SUMS)。PR #4保持Draft，不合并。

**唯一建议：审阅并冻结这次B功能性抓持证据，再单独确定柜顶放置的最小任务级验证；本轮停止，不增加其他模型、驱动或精度工作。**
