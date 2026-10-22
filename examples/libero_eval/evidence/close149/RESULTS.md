# Issue #5：close149 单变量真实实验

**唯一一次已执行：4793 / 最多6500物理步，在lift阶段因`LOADED_DEPTH_GROWTH`早停。未离桌、无合格持握，原生柜顶任务判定false；不重试。**

授权：[Issue #5 / 6295728385](https://github.com/LeoandDream/Nero_DH116_assets/issues/5#issuecomment-6295728385)。基线`c9de3e45e1f4dbccb7e8ebf659e5102aaec328ea`，先核查本地9份执行/claim记录、输出和进程，未发现本轮候选已运行；新输出目录和执行标记原子创建。单次运行从完整原生Reset开始，未重跑左手或原右手基线。

## 实际变量与最小接入

- `close_end_source_macro=149`：`old[:190]+old[200:]`，后70条手动作/目标角锁source149；不冻结实际qpos。260宏×25步、dt=2ms；前4750步输入不变；保留原40宏lift、30宏hold的全部臂目标和时长。
- 候选规范JSON SHA256：`ebf188b0bf49ad3b74a042c52b5fa4ad5df02dcb8f84caea565fcdaf9fdc7e32`。首次动作差异4751；原lift第一宏重复起点，4776才首次改变臂目标。
- 新入口[`../../../right_hand/close149.py`](../../../right_hand/close149.py)复用冻结研究运行器；原有文件均不改。实际execute只替换动作列表和日程摘要两行；另以独立入口记录6500步注册与新来源SHA，见[`raw/close149-executor.diff`](raw/close149-executor.diff)、[`raw/close149-entry.diff`](raw/close149-entry.diff)。
- 冻结81块手部、306块腕部、NERO/控制/Mimic/solver/过滤、`DH116_RESEARCH_CONTACT_V1`及全部独立保护不变。306腕仅保留以隔离动作变量，不认定为最终资产方案。政策SHA `48632dfa4bbe0630ef7d61b9e706240d7769ad6361d588e1d02025bde4bc6454`；跨close→lift不清空窗口。
- Python3.11.16、MuJoCo3.2.3、NumPy1.26.4、stock robosuite1.4.1、LIBERO`f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`。源码和固定依赖只读挂载、旧私人runs不可见，仅新输出父目录可写；实际argv、时间及退出2见[`raw/execution.json`](raw/execution.json)。

## 与原真实记录对照

| 项目 | 原270宏研究尝试 | 本轮close149 |
| --- | --- | --- |
| 计划 / 实际物理步 | 6750 / 4775 | 6500 / 4793 |
| 首次旧2mm FAIL | 4769 | 4787 |
| 首次研究停止 | 4775，close | 4793，lift，macro191/source161 |
| 同为4775步的finger23最大深度 / body载荷 | 2.614642mm / 5.152556N | 1.863310mm / 3.854507N |
| 最终相对Reset高度 | −69.962537mm | −69.908946mm |
| 最终相对本次沉降位置高度 | +0.626753mm | +0.680344mm |
| 离桌 / 合格持握 / 原生任务 | 未离桌 / 0s / false | 未离桌 / 0s / false |

前4750步state及完整contacts/RK4缓存**逐行原始JSON完全相同**，首次差异4751；Reset关键字段及formal/actual完整编译快照相同。只按原动作日程删除/锁定指定字段，260条映射与候选SHA全部核对通过，见[`independent-review.json`](independent-review.json)。

4793唯一触发为`gripper0_finger23_link ↔ white_bowl_1_main`的body聚合：4787–4793连续7个深超限样本，跨度12ms，深度2.017380→2.043724mm，净增0.026344mm；平均法向标量载荷4.019216N，高于冻结参考2.331547N；采样梯形载荷暴露0.0482305919N·s（非连续积分冲量）。最终body载荷4.040279N。

本轮完成settle250、approach2000、descend1750、close750、lift43步；其中真实臂目标开始改变后仅执行18步。hold未到达，原定1.5s保持未执行。4751–4793全部43步仍有正环境支撑，末步桌—碗法向力0.003272921N，不能把0.680344mm高度变化称为无支撑抬升。

## 分项结果与证据

- **独立数值/关节保护：保存前缀PASS。** 无非有限值、MuJoCo warning、意外腕部接触或路径包络失败；臂误差最大0.088807rad < 0.1，Mimic最大0.000245947rad < 0.02，限位超出0。hold控制未测试；不能以max_active_hold=0冒充hold通过。
- **研究接触判据：FAIL/正确早停。** 源`control_health=FAIL`包含该接触保护，不表示关节或数值发散。旧2mm判据独立保留，历史FAIL不改。
- **限定抓持：UNKNOWN_EARLY_STOP；原生柜顶任务：false。** 未执行后续lift/hold，不推断会抓稳。此次也没有新增柜顶放置动作或成功率统计。
- [`episode/`](episode/)为原统一评测导出的4793行、状态/action/ctrl、阶段/Reset来源、结果和相机索引。192组双相机、384张JPEG；[agentview视频](episode/videos/agentview.mp4)、[手腕视频](episode/videos/robot0_eye_in_hand.mp4)均192帧/20fps。逐帧仿真时间以cameras索引为准，视频固定帧率包含末尾不足一宏的观察帧；前4750步对应380张JPEG与原记录逐字节相同。
- [`raw/`](raw/)保留原始状态、接触、力、RK4缓存、冻结快照、实际执行源码及diff；大编译快照仅无损gzip，SHA见[`artifact-source.json`](raw/artifact-source.json)。post-step qpos/高度与最终RK4子阶段接触/力/body缓存仍明确分开，观察不追加forward。显式4793轨迹步不宣称覆盖编译器内部自检。
- 8项针对性纯离线测试通过，覆盖映射、阶段、目标锁定、SHA、退出码、既有输出保护及跨阶段政策历史；见[`tests.log`](tests.log)。全部旧受跟踪文件、main、左手V1、正式资产、统一接口、用户README保持不变，见[`verification.json`](verification.json)。

复现入口（实际运行需要新的单次授权和新输出，不自动重试）：激活原已锁定环境后`python -B examples/right_hand/close149.py --output runs/NEW/close149_once`。统一离线导出仍使用`examples/libero_eval/run.py --hand R --mode research --recorded-source ACTUAL_RUN --output NEW_EPISODE`。

**解释与唯一下一项建议：**停止继续增加手目标减轻了原4775步接触深度和载荷，旧/研究停止均延后18步，但未解决持续闭合增载或完成离桌。若另获授权，下一项最小动作变量建议仅删除lift首条重复臂起点宏（source160），让真正臂目标变化提前25步；保留source149手目标、其余空间路径与全部保护。该建议未实现、未运行，不保证能消除停止；不追加几何或腕部优化。PR #4保持Draft，不合并main。
