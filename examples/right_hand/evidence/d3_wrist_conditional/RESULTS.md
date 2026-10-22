# D3条件式诊断：越过旧腕部故障段，4742步闭合接触超限即停

按[6269815750](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6269815750)执行。实际运行至 **9.484s、close阶段第4742/6750步**（189条完整宏命令，下一条17步），原2mm交叠界限触发，退出码2；没有重试或补跑。原1315–1401步阶段已通过，当前阻塞转为finger22—碗壁接触，抬升/持握未执行。

| 分项 | 结果及边界 | 证据 |
| --- | --- | --- |
| 迁移/输入一致性 | **PASS**：冻结b7daa08的306腕部＋手78，零新增代理；原270条动作精确相同；完整原生Reset qpos/qvel/ctrl一致；369个Reset后对象、原物理/控制/Mimic/过滤核对无非授权差异；508项冻结输入前后一致 | `adaptation-audit.json`, `run/old-to-new-compiled-audit.json`, `run/frozen-before-first-step.json`, `verification.json` |
| 外物有限采样预检 | **PASS，限参考场景的921姿态**：实际碰撞资格下60357对/姿态，55520061项完整几何AABB证分离，68736项完整编译凸体投影/既有LP证分离；0阻塞，腕部—腕部查询0，未重复旧21,526,533项调查 | `preflight/summary.json`, `preflight/poses.json.gz`, `preflight/pair-inventory.json.gz` |
| 动态控制/物理健康 | **旧J6段通过；整体物理健康FAIL**：4742步最大臂误差0.0888066rad＜0.1；旧1315–1401段J6最大0.0877697rad；第4742步交叠2.014716mm＞2mm即停。Mimic最大0.0002700rad、越限0、数值warning0、全部有限；hold误差未验证 | `run/result.json`, `run/state.jsonl.gz`, `review-source.json` |
| 限定抓持 | **UNKNOWN_EARLY_STOP**：4518步首次正力手碗接触；有80步拇/食对向接触，但碗仍受桌支撑、有效hold 0s；尚未lift/hold。native=false、strict=false | `run/contacts.jsonl.gz`, `review-source.json` |
| 数值附查 | **FAIL原样保留**：原9.615nm逐块raw/compiled附查及1e-10m标准不变，不能当完整源面误差上界；未批准0.300mm或2.17mm例外 | 原`../d3_wrist_reuse/compiled-numerics.json` |
| 发布 | **未批准/未合并**：不签发整包高保真PASS；原1401步与7000步FAIL全部保留 | Git差异与`verification.json` |

**风险区预检与实际接触。** 完整法兰123块（root_arc_1）/103块（profile_2_011）对手的最小证书下界分别19.071/32.844mm，对参考碗91.655/113.096mm，对桌126.124/147.566mm。使用完整块而非单个点；没有把2.17mm当全局误差上界。静态碗位只作规划参考，运行完全原生Reset并沉降，没有注入位姿。实际4742步中新增腕部与手/桌/碗/其他部件接触计数为0；正J6边界域、未执行后半程及未覆盖物体仍UNKNOWN。

**首失败局部证据。** `white_bowl_1_g22 ↔ gripper0_proxy76_finger22_link_00`，法向力1.719859N，native距离−2.014716mm；第4741步尚未超限。取该步原始RK4缓存body位姿，厂家finger22 STL与该原生碗壁box完整三角面间隙 **0.329680mm**，未相交；交付raw STL亦为0.329687mm。raw代理整体凸包与box的独立LP内球半径 **0.841423mm**，支持凸包封口/填空接触，不能将native交叠写成源面穿透深度。源STL存在81条非流形边，box中心/角点绕数接近0仅作包含诊断，硬件材料实体结论仍UNKNOWN。没有展开其他部件审计或修改手部。

旧/新前1314步的qpos、qvel、ctrl、actuator force及动作逐值一致；首差异1315步仅full_qvel，对应旧腕部首次接触。第1401步新J6误差0.087770rad，旧为0.111590rad。该结果支持本限定运行越过旧腕部阻塞，不证明整个控制域或整包模型正确。末态高度相对Reset **−70.222273mm**、相对本次自身沉降 **+0.367017mm**，两种基准不混用。

**唯一下一步：**提交finger22单凸代理封口的局部静态修复预览，以本次保存的4742步触点、源面间隙和闭合段作可证伪约束，保留真实指面；等待审查后再决定实施。优先不动已跑过故障段的腕部，不改控制/过滤/阈值来绕过当前FAIL。

**执行透明性。** 积分前曾有一次隔离挂载点错误、一次把旧序列化XML与Reset后模型混比的检查器错误，均保留在`isolation-startup.*`与`setup-block-before-integration/`，物理步均0。修正后改为旧/新实际Reset快照对比，未忽略body位姿，完整一致；只进行一次真实积分，无停止后重跑。`query-debug/`保留初稿大distmax返回假负距离的反例及独立投影复核；最终预检不用该查询作判据。一次因BLAS多线程过量中断的纯静态初稿未产生动力学或改模。没有把这些检查器修正回写成旧模型PASS。

模型SHA256仍 `9604a4a6935a34496f93e223219f687b4df8b7aa83037dc208fc9f3499eb05c9`；实际runner SHA256 `c518aa6b1df4c3725b3c568a21528d23a295753762619295c6cb608ec819b990`。Python3.11.16/MuJoCo3.2.3/stock robosuite1.4.1/NumPy1.26.4，LIBERO f78abd68ee283de9f9be3c8f7e2a9ad60246e95c。命令及只读隔离挂载见`execution.json`；已有原生forward数量9489、Reset刷新step1两次，未为记录添加forward。接触位置、法向、力和body位姿是RK4最后内部子阶段缓存；qpos/qvel与物体高度是积分后状态；J6广义驱动/约束量直接读缓存，不重算求解器。

20项针对性静态测试通过；1026个原文件中仅授权runner发生变化，正式左右手、共享资产、旧候选/FAIL、main及用户README未改，PR保持Draft。所有原始状态/接触保留压缩JSONL；物理快照仅在发布副本中无损gzip，原运行目录原文件仍保留。

只做离线复核，**本次唯一动力学额度已用尽**：

```bash
python -B -m unittest discover -s tests -p 'test_wrist*.py' -v
python -B -m unittest discover -s tests -p test_white_bowl_frozen78.py -v
python -B -m unittest discover -s tests -p test_right_white_bowl_model.py -v
python -B examples/right_hand/evidence/d3_wrist_conditional/review.py --output runs/wrist-conditional-review.json
sha256sum -c examples/right_hand/evidence/d3_wrist_conditional/SHA256SUMS
```
