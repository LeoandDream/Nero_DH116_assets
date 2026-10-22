# 81手部 / 306腕部：唯一一次白碗探索性动力学

任务：[PR #4 / 6273337682](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6273337682)。基线 `685c458`。**实际运行到4731步，在close阶段按新增UNKNOWN规则早停，退出码2；4742未到达，lift/hold及抓持UNKNOWN。一次积分额度已用尽，没有重试。**

| 项目 | 本次结论与证据 |
|---|---|
| 模型/输入一致性 | **PASS**：冻结模型SHA `7f9d84061c3e7d176763c688d2389f8b3d3e38386b8e990a0fa047ff2bc3329c`，手81/腕306。XML及所有资源归一比较、独立compiled字段比较、实际Reset快照674对象比较均通过；只允许finger22单块→四片。原270条13D日程逐值一致，完整Reset的qpos/qvel/ctrl/JPC初始参数与旧运行一致。见 `static-input-audit.json`、`run/old-to-new-compiled-audit.json`、`review.json`。 |
| 原动态阈值 | **已执行区间PASS，完整日程UNKNOWN**：臂最大误差0.088806594rad、Mimic 0.000245247rad、越限0、最大合资格交叠1.254730mm、warning0、状态有限。非hold手主动误差最大0.042415rad；原0.02rad门槛只用于hold，本次尚未验证hold。J6旧1315–1401区间通过，最大误差0.087769749rad。 |
| 新风险停止检查 | **UNKNOWN早停**：4731步 `white_bowl_1_g22 ↔ gripper0_finger22_local_01`，原生交叠0.004886mm、法向力0.241743N。风险面归类返回空候选，立即停止。腕部非预期外物接触0；未确认根部或内凹封口面异常。原始runner的`control_health=FAIL`包含这次风险UNKNOWN，不应解释为关节跟踪超限。 |
| close / lift / hold | close实际731/1000步；lift、hold各0步。4742 **UNKNOWN_NOT_REACHED**，未证明原4742故障已消除。 |
| 限定抓持 | **UNKNOWN_EARLY_STOP**：4518首次正力手碗接触，49步拇食对向接触；末态仍有桌面支撑，合格hold 0s。相对Reset **−70.359239mm**；相对自身250步沉降基线 **+0.230051mm**。native=false，strict=false。 |
| 数值/几何与发布 | 原0.1nm数值**FAIL不变**，未批准孔腔、腕部0.300mm/2.17mm外填例外，非流形材料仍UNKNOWN；没有正式高保真或发布PASS。原7000、1401、4742步历史FAIL均保留。 |

**首个差异。** 与360e53a旧真实运行相比，前4662步的qpos/qvel/ctrl/actuator force及动作逐值一致。4663步旧单凸块开始接触，qpos/qvel/actuator force首次分叉；ctrl于4664步首次分叉。原动作始终一致。当前四片首次finger22接触为4731步，不能把接触延后等同于抓持改善。

**归类器限制，不能算成模型的新几何FAIL。** 只对保存的4731步实际RK4缓存离线核查：接触中点距实际编译mesh_graph边界2.630314µm，超过预注册的`abs(dist)/2 + 3.033nm`定位范围2.446004µm，差0.184310µm，因此候选面为空。最近凸面法向 `(0.986569,0.145350,0.074531)` 与接触法向在finger22坐标系内相符，位置靠近+X功能前表面，不是已确认的根孔或内凹底面触碰。分类器的半穿透距离定位假设没有认证MPR接触点位置误差；原CCD tolerance为1µm，但这本身不是误差界证明，不能事后加入范围继续跑。

厂家完整finger22 STL与该完整原生碗壁box在这个缓存姿态的表面间隙 **42.765110µm**，最近源面6175，无三角面相交；与当前凸代理的微小接触分开报告。非流形源的完整材料包含关系仍UNKNOWN。此检查没有替代真实积分，也没有重扫742/921姿态。

**记录与运行隔离。** Python3.11.16 / MuJoCo3.2.3 / stock robosuite1.4.1 / NumPy1.26.4 / LIBERO `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`。只启动一次实际试验，原生constructor和显式Reset各1次，`mj_step=4731`，Reset非积分`step1=2`，原生`forward=9467`；记录器没有追加forward或重算控制。`execution.json`保存完整命令及开跑前runner/policy/protocol SHA；运行代码只读挂载，旧工程树隐藏。依赖环境与输入网格没有重建。

`run/state.jsonl.gz`逐步保存post-step状态、动作、ctrl、驱动力、五条Mimic残差及缓存广义力。`run/contacts.jsonl.gz`逐步保存接触geom/四片身份、位置/法向/力及**全部机器人body和白碗的原始RK4末内部阶段位姿**。这些缓存不是post-step FK，不能混用。14份原运行文件完整保留，仅两份JSON无损gzip；原文件SHA见 `run-source-files.json`。

**唯一下一步：先修正仅用于诊断的MPR触点归类方法，在已保存4731步缓存上验证功能面与风险面的可靠区分，再申请新的单次动力学授权。** 不修改模型、动作或物理阈值，不事后改判本次停止，不自动补跑。

本提交保留实际执行的runner和policy原字节，包含归类器的上述限制；这是实验诊断入口，不是发布验收入口。保护正式左右手、共享资产、main和用户README；PR继续Draft。

离线复核（不积分）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -B \
  examples/right_hand/evidence/d3_finger22_dynamic/review.py --output /tmp/finger22-dynamic-review-new.json
sha256sum -c examples/right_hand/evidence/d3_finger22_dynamic/SHA256SUMS
```

实际执行入口为 `white_bowl_run.py --finger22-conditional --output runs/d3_finger22_four_once`；精确隔离命令见 `execution.json`。**该额度已用，不得自动再次执行。**
