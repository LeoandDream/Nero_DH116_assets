# 唯一一次白碗探索运行：通过4731/4742，4769原物理阈值早停

任务：[6290062555](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6290062555)，基线 `16b851f`。本次**确实执行一次、共4769次mj_step**；不是静态回放，也没有重试。结果为 `EXPLORATORY_STOPPED_AT_FIRST_ORIGINAL_PHYSICAL_BOUND`，限定抓持 **UNKNOWN_EARLY_STOP**，正式发布未批准。

**做了什么。** 新建独立 `white_bowl_exploratory.py`，原字节复用旧执行函数。仅显式选择探索性停止策略、冻结模型和记录元数据；原runner/严格归类工具未覆盖。保持81手、306腕候选、270宏步×25步（最多6750）、原13D动作、seed20261007、原生完整Reset、JPC及全部物理配置。在线不运行严格面域证明，接触标签只记 `DEFERRED_OFFLINE`；原物理健康、腕部异常接触及规划包络停止全部保留。

| 核查 | 实际结果 |
|---|---|
| 真实执行进度 | settle250、approach2000、descend1750、**close769/1000**步；190条完整命令，宏步190中执行19步。**lift0、hold0**，未继续或补跑。 |
| 4731/4742 | 两步均真实越过。前4731步的状态、动作/ctrl、驱动力及原生接触/力/RK4 body缓存与上次81/306运行逐值相同；仅停止/归类元数据不同。取消归类早停没有改变此前物理轨迹。 |
| 首个停止 | **4769：ROBOT_CONTACT_OVERLAP**。唯一超2mm对为 `white_bowl_1_g23 ↔ gripper0_proxy76_finger23_link_06`（finger23，非finger22）：**2.029397981mm，Fn=1.737957894N**。同对4768为1.929396020mm、1.701058596N；4768全局最大是g22↔同一finger23代理的1.946405394mm。 |
| 原健康指标 | 最大臂误差0.088806594rad＜0.1；Mimic0.000278909rad＜0.02；关节越限0；warning0、状态全部有限、腕部异常外物接触0。非hold主动误差最大0.046661215rad，不误套仅hold适用的0.02rad。hold未执行，其追踪尚未验证。 |
| 白碗与抓持 | 4518首次正力手碗接触；末态仍有正力桌面支撑，沉降后无任何无支撑步。相对Reset **−70.010505mm**，相对本次沉降 **+0.578785mm**，合格连续hold **0s**。不能称为离桌或抓持成功。 |
| 评价信号 | `B=FAIL`、`control_health=FAIL`（接触阈值，非关节跟踪超限）、`grasp_result=UNKNOWN_EARLY_STOP`、`native_task_result=false`；`strict_grasp_success_claim=false`只表示未宣称严格成功。 |
| 一致性/次数 | 518份原执行输入SHA一致；完整Reset状态/控制器、270条动作及原生编译物理快照精确相等。constructor1、显式Reset1、mj_step1非积分刷新2、原生forward9543、mj_step2=0。forward为原环境Reset/控制流程，没有新增记录用forward。1170份旧跟踪文件及用户README保持SHA一致。 |

**事后严格分析独立保存。** 运行结束后，用冻结 `16b851f` 归类器处理全部75项实际finger22接触：4731的一项为已注册局部功能面近似，另74项 **UNKNOWN**。这些离线判断没有写回原轨迹、没有影响本次停止。4731仍记录原生代理距离−4.885942µm、厂家源面—碗壁间隙+42.765110µm；没有扩大功能域或降低精度标准。后处理禁止执行API尝试0，新增积分/Reset/FK/原生碰撞求解0。

**解决与未解决。** 本次解决了严格面域UNKNOWN阻断探索运行的问题，识别出随后真正触发的原2mm代理交叠约束。它没有修复几何，也未证明finger23厂家STL真实实体相交；该源几何问题仍UNKNOWN。原0.1nm数值FAIL、非流形材料/全手保真UNKNOWN、腕部0.300/2.17mm风险及全部历史FAIL保留，不把本次结果写成高保真验收。

**唯一下一步：只读复核4768/4769保存缓存中的finger23_link_06与原生g23，区分代理外填/凸封口与真实源面接触，提出最小修复建议。** 不授权自动修模、调参或下一次积分。PR保持Draft、不合并main。

证据：`execution.json/log`（唯一实际命令/退出码/源SHA）、`registered-protocol.json`、`run/state.jsonl.gz`与`contacts.jsonl.gz`（原始压缩字节）、`run/result.json`、`run-source-files.json`（16份原始产物；编译快照仅无损gzip）、`review.json`、`selected-states.json.gz`、`posthoc-finger22.json.gz`、`physical-validation.json`、`tests.log`（7项零动力学针对性测试）。`review.py`从保存日志独立复算全部4769步的原物理阈值、包络、时钟和支撑，仅4769首次超限。

安全复核现有证据（不执行新积分）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data_robot/verification/nero_dh116_right/.reference-env/bin/python -B \
  examples/right_hand/evidence/d3_finger22_exploratory/review.py \
  --output runs/finger22-exploratory-review-new
sha256sum -c examples/right_hand/evidence/d3_finger22_exploratory/SHA256SUMS
```

真实运行命令已原样保存于`execution.json`，**本授权次数已耗用，不得再次运行或改名补跑**。
