# D3：两对内部接触排除，单次7000步对照

- **做了什么：**以`6b43273`为基线，复用原7000步对照，只在临时诊断模型中排除A、B，新运行一次7000步；正式模型未改。
- **发现什么：**finger11稳态跟踪恢复，另外三路联合闭合仍超限；剩余实际接触最大报告交叠反而增大，候选仍FAIL。
- **解决了什么：**支持这组额外内部接触是部分控制失准的原因，排除了“只是接触计数归零”的解释；没有修复正式资产，也没有区分A、B各自贡献。
- **下一步：**只读归因剩余联合开合接触，重点检查首次失效6242步的finger13↔finger42，以及最深6395步的finger12↔finger43；先形成修复提案，不继续逐条关碰撞。

依据[执行授权](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6251405446)。固定基线`6b432737dbddc7e8917630382c582ac8704d637a`，实际执行入口提交`f068d125728b4f6d2ba0f443c303eb8b7705590a`。

## 干预范围和实际执行

A=`gripper0_base_link↔gripper0_finger12_link`；B=`gripper0_finger12_link↔gripper0_finger42_link`。仅向临时XML新增这两条body级exclude，原NERO exclude保留，17个手部collision geom仍为1/1。源XML其余字节不变。完整比较572个公开编译字段，差异仅exclude和匿名名称存储元数据；5671个geom pair仅上述两对改变资格，116523个独立假想外物mask用例无变化。body/weld映射没有扩大排除范围：[编译字段与资格](compiled-comparison.json)。外物接触资格只做结构核对，未增加物体或探针。

在独立克隆`/data_robot/verification/nero_dh116_right`、原固定环境执行；bubblewrap隐藏`/data_robot/projects`和旧`/tmp`：[隔离记录](isolation-exit.json)。原环境、协议、96个源输入、Python SHA、58个版本及6个核心wheel共4265个包文件均核对一致：[基线审计](baseline-audit.json)。原7000步未重跑。

原λ=0的全部动作、目标、相位、稳态窗口、时间逐值相同，不使用λ=1；Reset/keyframe和JPC初始化源码、线性Mimic、所有机械/接触/控制参数及阈值冻结。独立attempt/completed计数、原run_case计数、状态/接触行数均7000，仿真14.000000000001345s，运行前后所有编译字段不变：[Reset及步数](reset-and-step-audit.json)。原历史没有完整integration/cache快照，不能宣称隐藏初态逐字节核对；已确认原生keyframe、首qpos/qvel/ctrl/time及初始化顺序一致，并保存本次完整Reset/integration快照。

## 真正的控制效果

以下是原先约定稳态窗口中的最大绝对误差；原界限均为0.02rad。全程瞬态不适用该稳态阈值。

| 通道 | 原7000步/rad | 候选7000步/rad | 候选结果 |
|---|---:|---:|---|
| finger11 | 0.044222741 | 0.011368755 | PASS，越界100→0步 |
| finger12 | 0.097557312 | 0.064242800 | FAIL，越界50步 |
| finger21 | 0.008375810 | 0.008375890 | PASS，基本不变 |
| finger31 | 0.132974921 | 0.122632153 | FAIL，越界50步 |
| finger41 | 0.215341763 | 0.109788282 | FAIL，越界50步 |
| finger51 | 0.008348368 | 0.008348368 | PASS，不变 |

finger11单独闭合稳态误差由0.024062332降至0.000785429rad；联合闭合仍存在另外三路超限。两条轨迹前199步qpos/qvel/actuator_force严格相同，首差均为第200步/0.400s，与基线A首次报告接触同一步；JPC机械臂力矩首次改变在201步，是相同控制器对不同物理响应的反馈。[首差和完整指标](audited-comparison.json)、[对比图](comparison.png)。

| 其他指标 | 原7000步 | 候选7000步 |
|---|---:|---:|
| 全阶段最大主动误差/rad（含瞬态） | 0.383836254 | 0.400739893 |
| 首次启用交叠超过2mm | 229步/0.458s | 6242步/12.484s |
| 首次稳态跟踪失败 | 451步/0.902s | 6451步/12.902s |
| 最大实际启用报告交叠/mm | 12.477555 | **14.197594** |
| 去掉A/B后同口径剩余交叠峰/mm | 12.296655 | 14.197594 |
| finger12↔finger43峰值法向力/N | 63.324909 | 75.851828 |
| arm最大保持误差/rad（界限0.1） | 0.027465712 | 0.027630006 |
| 最大线性Mimic残差/rad（界限0.02） | 0.000687164 | 0.000695273 |
| 最大关节越限/rad（界限0.01） | 0.000123283 | 0.000000771 |
| 非有限状态／MuJoCo数值warning步数 | 0／0 | 0／0 |
| 整体数值筛查 | FAIL | **FAIL** |

候选最早剩余失败是6242步的finger13↔finger42；全程交叠超限378步，联合闭合稳态失败6451–6500共50步。最深交叠14.197594mm来自6395步finger12↔finger43。该对仍真实产生333步接触，证明过滤未扩展到finger42的焊接邻居finger43。其他接触对和峰值均在audited-comparison中保留。稳态部分改善不能表述为所有跟踪指标改善：finger31的瞬态峰反而增大。

## 原记录器勘误：保留raw，单独校正统计

原`run.py:filter_reasons`除真实body signature外，还查询weld signature。新增B排除后，它把finger43(body23、weld22)误当成finger42(body22)一起排除；MuJoCo 3.2.3引擎只匹配实际body signature。因此333条finger12↔finger43接触被错误标记`enabled_nonadjacent_robot_self=false`，163条state的交叠汇总低估。[固定引擎源码](https://github.com/google-deepmind/mujoco/blob/476599f8b6175ef5a3040c47ebb3cd99a83c5365/src/engine/engine_collision_driver.c#L333-L344)。

**这是记录分类错误，不是引擎过滤外溢；真实控制/轨迹未改，也未重跑。** 原始[filtered-result.json](filtered-result.json)与[legacy-comparison.json](legacy-comparison.json)中的6.767585mm漏算了该对。权威汇总采用[audited-comparison.json](audited-comparison.json)的14.197594mm；[逐步独立校正](audited-contacts-per-step.jsonl.gz)保留每步更正及错标明细。原始state/contacts/result均逐字节保存。校正前后整体FAIL、首次失败和失败步数不变；不能直接采用旧6.767585mm汇总。

离线分析禁止step/forward/积分调用，新增物理步0；新增一项基于真实旧接触记录的回归测试捕获该weld邻居误标。3项编译/范围测试与该1项测试通过。独立复核按实际body重新计算全部7000步，与公开校正逐值一致：[预执行审查](preexecution-review.json)、[执行后复核](postexecution-review.json)。没有运行旧完整测试入口、2000步探针、LIBERO或网格全扫描。

## 使用与证据

仅在明确需要重新复现时执行第一条；本轮已经执行一次，没有重试。先按上级固定环境说明安装并激活，然后分别执行：

```bash
source examples/libero_reference/env/activate.sh
python -B examples/right_hand/internal_contact.py --output runs/internal_001
# 第一条本次实际退出2：7000步完成，数值筛查FAIL；不要自动重试。
python -B examples/right_hand/internal_contact_analysis.py \
  --run-output runs/internal_001 --analysis-output runs/internal_audit_001
```

`--preflight-only`可仅检查环境/编译范围、不forward或步进。`diagnostic.xml`是实际临时XML的证据副本，不是正式资产；其相对资源基目录为仓库`mjcf/`，应由诊断入口按原基目录编译，不能直接以本证据目录为资源基目录加载。

- [真实状态](filtered-state.jsonl.gz)、[真实接触/力](filtered-contacts.jsonl.gz)、[实际7000动作](frozen-commands.json.gz)、[命令和退出码](execution.json)。
- [输入身份](input-identity.json)、[环境](environment.json)、[编译对照](compiled-comparison.json)、[原协议](frozen-protocol.json)、[临时XML](diagnostic.xml)。
- [文件/压缩/解压SHA与行数](artifact-index.json)、[独立核验](verification.json)、本目录`SHA256SUMS`。原对照仍引用上级历史证据，未复制或修改。

A/B状态为 **FILTERED—原始STL表面相交风险依然存在**。接触归零不是几何PASS。本次结果仅支持该局部接触策略改善部分控制，候选仍FAIL；正式右手7000步验收保持FAIL，抓持UNKNOWN。全部原408个tracked文件、左手V1、正式main与未提交README保持原样。唯一后续建议见开头；未增加第三条exclude，未合并main。
