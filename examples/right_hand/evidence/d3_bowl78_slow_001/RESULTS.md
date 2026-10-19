# 78块右手白碗：唯一减速尝试，提前停止

做了什么：固定 `finger13_local` 78块模型，原230条动作数值及顺序不变，仅把40条 `approach_high` 各执行两次；预算270宏步、6750物理步。完成最小入口/角色适配后，实际只运行一次。

发现什么：第1401步、2.802s，关节6跟踪误差0.111590187rad，首次超过原0.1rad界限，立即停止。此前1315步起已经出现机械臂link5—法兰接触；没有手—碗接触。

解决什么：78块模型的原生LIBERO运行链已接通并由真实运行验证；减速方案仍未通过接近阶段，不能交付右手白碗抓持PASS，也不能唯一归因于控制器、速度或代理。

唯一下一步：先只读回查保存的1315至1401步，比较link5—法兰源几何与代理的间隙、接触力矩及J6跟踪时序，判明真实干涉还是代理假接触，再决定修复范围。本轮没有继续运行或修改模型。

## 冻结与接入

授权：[PR #4 / 6260545305](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6260545305)，代码起点 `a949ebe9be7f33ebb4d6e89a8604dbd9b265027b`，候选源 `e7123c0`。

- 唯一机器人：`examples/right_hand/candidates/finger13_local/model.xml`，SHA256 `b27f4f86fbaeacd2bb925463f39f67181ef7db12963c4d9f7fc01797a7924d1b`；没有回落正式17块入口。
- 原路径：`examples/right_hand/evidence/d3_white_bowl_001/candidate.json`，SHA256 `ba767b42f99d3562255909bde5c970d529ba254769029ec1cd7a98faf76af0fb`。新日程见 `schedule.json.gz`，逐条保存actual/source macro及重复序号。
- 手78、臂51、法兰8，另保留挂载座3个碰撞体；仅原base—link1 exclude，无诊断exclude或显式geom pair。四个robosuite接触角色按原body语义映射，数量15/40/12/36。
- 2454项非手部碰撞替换范围之外的编译字段精确一致；实际LIBERO组合模型与78源模型4083项命名物理字段/mesh载荷核对通过。921姿态有限采样证据明确绑定78模型，仅支持采样路径的手内预检。
- Python3.11.16 / MuJoCo3.2.3 / stock robosuite1.4.1 / NumPy1.26.4 / LIBERO `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`；原生场景、seed20261007、线性Mimic、控制器/servo/接触/物理参数和全部验收阈值保留。
- 原生构造1次、底层Reset1次，Reset全qpos/qvel/ctrl与旧记录精确相等；积分前冻结模型、资源、动作、角色、控制器与211项输入SHA，运行前后一致。运行期间隐藏整个 `/data_robot/projects`，代码只读挂载，198条资源引用/192份唯一资源均位于独立运行目录内。

## 实际结果

| 检查 | 结果及范围 |
| --- | --- |
| 执行/控制 | 56个完整宏步，下一宏步1步；1401/6750真实mj_step；`STOPPED_AT_FIRST_FAILURE` / **FAIL** |
| 首次超限 | actual macro56，source macro33，repeat0，`approach_high`，J6误差0.111590187rad |
| 直接触发 | step1400误差0.097469992rad；下一目标变化−0.014119824rad，实际位置仅变化+0.000000371rad。仅记录值分解，不是动力学反事实 |
| 接触 | 1315–1401步link5_4↔flange_part_2持续正法向力；最大0.446109N、交叠0.0228379mm；停步仍有接触；手内、手碗、机器人外物接触均0 |
| 其他界限 | 最大Mimic残差9.82354e−7rad；限位越界0；无MuJoCo数值warning、记录状态均有限；臂ctrl无±100Nm触界 |
| 尚未执行 | 下降、闭合、抬升、hold；hold误差/抓持/实际指腹接触保真度 **UNKNOWN** |
| 两种高度 | Reset Z=0.97m；本次step250沉降Z=0.899410710103m；最终Z=0.899410957612m；相对Reset **−70.5890424mm**，相对自身沉降 **+0.000247509mm** |
| 抓持/任务 | 有效持握0s，碗仍由桌面支撑；`UNKNOWN_EARLY_STOP`；`native_task_result=false`，`strict_grasp_success_claim=false` |

与旧17块首跑对照：Reset及前275步full qpos/qvel、action、ctrl、actuator force逐值相同；首次差异step276恰为获准重复动作开始与旧日程不同。原501步FAIL保留；本次首次超限前存在机械臂—法兰接触，根因 **UNKNOWN_CONTACT_CONFOUNDED**。此次同时更换碰撞候选与时间参数，不是相对旧17块的单变量因果实验。原7000步全幅FAIL不变。

接触位置、法向、力和body变换来自RK4最后内部阶段缓存；状态/高度来自积分后qpos，没有追加forward。对向接触判定保留旧计算口径，包含该时间对齐限制。没有观察到手—碗接触，不能验证预期14/23指腹区域，也未据此宣称代理高精度。

## 入口与证据

本次运行入口如下；唯一尝试额度已消耗，没有再次执行：

```bash
source examples/libero_reference/env/activate.sh
python -B examples/right_hand/white_bowl_run.py --frozen78 \
  --output runs/d3_bowl78_trial_001
```

`execution.json`保存实际隔离命令、时间和退出码2；`state.jsonl.gz` / `contacts.jsonl.gz`为1401步原始记录；`model.xml`、`arm.xml`、`hand.xml`、`mount.xml`为实际组合输入原字节，绝对路径仅作为来源证据，运行时由入口重新装配。`frozen-before-first-step.json`和压缩的`robot-physical-preflight.json.gz`记录开跑关口。

`independent-postrun-review.json`、`contact-failure-review.json`和`independent-control-review.json`只读重算原始记录；`verification.json`与`SHA256SUMS`绑定完整证据。qacc未单独记录，其有限性由运行时finite检查标志记录。8项纯静态测试通过，未运行会增加物理步的全量测试。588份既有文件、左手V1、正式右手、旧候选/FAIL、main及未提交README均未改变；未合并main。
