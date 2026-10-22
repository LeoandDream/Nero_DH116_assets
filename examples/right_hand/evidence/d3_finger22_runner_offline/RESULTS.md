# 冻结局部来源规则：离线运行级接入通过，尚未授权动力学

任务：[6283942006](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6283942006)，基线 `cab4305`。
**结论：`OFFLINE_INTEGRATION_PASS_READY_TO_APPLY_ONLY`。可以申请下一次探索性单次实验；本轮没有启动或授权实验。**

| 检查 | 实际结果与边界 |
|---|---|
| 冻结规则 | `frozen-rules.json` SHA256 **`f5e837f43332366af3ac964accd6107b4d58772cd185fbc16979fb1acf6f5800`**。引用原源面/邻接/全源外露证书；每个候选仍须重新证明完整局部投影覆盖、外侧关联及全源包络。没有 +X、face285 或整指面白名单。 |
| 全部已有缓存 | 回放 **4731行、39,734个接触事件**，包含全部已保存的 finger22—碗事件：只有4731的一项。其余 **39,733项原记录、资格和其他手指归类均原字节保持**；新接口返回 NOT_APPLICABLE。没有用 post-step qpos/FK 生成缓存。 |
| 4731 | `local_01↔g22`，唯一实际相交子片、独立凸壳面285，重新确认源6175局部功能面近似；公共球半径 **1.052624µm**。完整面285独立检查仍为切缝风险，**未整体放行**。原生 `dist=−4.885942µm` 与完整厂家源面—g22间隙 **+42.765110µm**分别保存；+X代理外扩48.313–49.025µm仍存在。 |
| 风险/未知停止 | **21/21**确定性负例符合预期：完整冻结凸体对合成box的根孔/凹腰封口、冻结切缝面片、未登记+X、边/点、多候选中含风险或UNKNOWN、空候选、源包络无法证明、姿态/资格/几何身份缺失或不符、错误阶段、非数、证明异常。合成面片、聚合与错误注入均单独标注，不冒充原生接触。 |
| 运行边界 | **19项测试通过**（12项新接口+7项既有几何测试）：实际geom/body绑定与原资格函数复算、双记录RK4姿态一致性、旋转/接触基格式、geom顺序交换、零力接触不跳过、全候选风险优先、异常停止、日志回填可序列化、原物理停止原因追加保留。初始化接口用4731字段构造事件结构测试，**不是重新Reset或原Reset缓存重演**。 |
| 开销 | 编译后的接口准备 **9.464s**；4731完整判定/源面间隙查询 **0.700s**；整次离线回放及负例 **20.311s**。这是同步离线诊断，不具备500Hz实时能力；未来固定步长仿真会增加墙钟耗时，不能据此减少检查频次、改变动作时钟或异步放行。 |
| 保护与物理调用 | **1159份旧文件全部SHA一致**，包括旧runner、旧归类器、4731历史早停、81手/306腕资源。用户README未变。所有 `mj_*`执行API（除身份查询）及 `MjData` 在回放/测试中被拦截，禁止调用尝试 **0**；新积分/Reset/FK/原生接触求解均 **0**，不重扫742/921姿态。 |

**最小接入实现。** 新增独立 `examples/right_hand/finger22_contact_adapter.py`：已完成Reset的原生编译模型交给 `ContactAdapter(model)` 一次性核验；逐接触调用 `classify_event(record, timing, step)`，或把原缓存行交给 `classify_step(cache)`；结果可安全附加到事件日志。`stop_reasons(decisions, existing_reasons)`只追加诊断停止，不替换原物理或腕部停止。模型、规则/输入SHA、实际编译字段、支持顶点均有校验，全部候选保留。

**以后经单独授权的实验入口所需接线（本轮未实施）：**
1. 在旧runner预积分准备位置，用新接口替代旧 `prepare_regions`；完整Reset、模型核验和原协议保持原样。创建独立实验入口，不覆盖旧文件。
2. 在初始缓存及每次真实步进后、任何额外forward之前，把原始事件字段和实际RK4缓存传给接口；附加诊断结果。初始事件使用明确的初始阶段标签，禁止冒称RK4。
3. 原数值/物理阈值及 `wrist_conditional.extra_failures`保持；用 `stop_reasons`追加新诊断停止，替代旧finger22面定位停止。先完整写出状态/接触/证明，再沿用首次失败即停。未登记、不支持或异常均停，无自动重试/降级。

离线加载发现旧XML尚未记录Reset写入的灶台/置物架位姿。本轮仅把**已有预积分快照**中的这两个固定场景body_pos/body_quat恢复到临时内存模型，再与完整保存编译字段逐项精确比较；未写XML、未调用Reset/FK。测试还查出并修正诊断对象回填形成JSON循环的接口缺陷；冻结规则SHA始终未变。

**尚不能宣称什么。** 历史4731 UNKNOWN早停不改写；4742、完整close/lift/hold及真实抓持尚未验证。原 `1e−10m` 数值 **FAIL**、源面提前接触、非流形孔腔材料及全指/全手保真 **UNKNOWN**、腕部0.300/2.17mm风险全部保留，未批准精度例外。静态证书和接口通过不构成新的动力学或正式发布PASS。

证据：`replay.json.gz`（唯一实际事件的完整候选、证书、原生字段；负例及整面285结果）、`summary.json`、`tests.log`、`verification.json`、`protected-before.json.gz`。原始缓存在旧 `d3_finger22_dynamic/run/contacts.jsonl.gz`，未重复复制。

复核（现有固定Python3.11.16/MuJoCo3.2.3/NumPy1.26.4/SciPy1.15.3环境，LIBERO资源位于原verification依赖前缀）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data_robot/verification/nero_dh116_right/.reference-env/bin/python -B \
  examples/right_hand/evidence/d3_finger22_runner_offline/replay.py \
  --output runs/finger22-runner-offline-new
/data_robot/verification/nero_dh116_right/.reference-env/bin/python -B \
  -m unittest discover -s tests -p 'test_finger22_contact*.py' -v
sha256sum -c examples/right_hand/evidence/d3_finger22_runner_offline/SHA256SUMS
```

**唯一下一步：审查本轮证据后，单独决定是否授权使用冻结规则的一次白碗探索性动力学。** 本轮到此停止，PR保持Draft、不合并main。
