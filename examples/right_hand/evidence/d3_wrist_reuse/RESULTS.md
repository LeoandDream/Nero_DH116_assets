# DH116腕部整部件复用：静态回归完成，尚未批准正式替换

做了什么：按[6268804256](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6268804256)，在18497ba的finger13_local上原字节接入既有306份OBJ，link5原7→147、法兰原8→159。发现什么：指定路径的435个旧异常全部消除，1401后态中87个腕部接触状态归零，未引入新接触。解决的是**已采样腕部代理假接触**，尚未证明J6控制恢复或白碗抓持。435个对照姿态来自18497ba已存的5块局部候选调查；本新模型的装配仍从finger13_local生成，没有沿用5块模型。

| 项目 | 结果与适用范围 | 证据 |
| --- | --- | --- |
| 迁移/字段一致性 | **PASS**；306/306原字节一致，零geom位姿；旧15块移除，手78保留；XML非授权内容精确一致；独立模型153、完整场景262个未替换geom及命名物理/控制字段一致 | `resources.json`, `xml-audit.json`, `compiled-audit.json`, `scene-compiled-audit.json` |
| 原921姿态腕部静态回归 | **PASS**；每姿态147×159，共21,526,533子对；宽相21,525,267项证分离，其余1266项投影证分离；严格重叠0、UNKNOWN 0。旧435消除435、残留0、新增0；旧649事件不与新子对作减法 | `plan-summary.json`, `plan.json.gz`, `certificates.json.gz` |
| 原1401后态静态回归 | 腕部接触状态87→0；新增其他body对0；全部1401状态的非腕部接触记录逐项一致，新腕部代理与手/桌/碗等接触均0 | `saved-summary.json`, `saved-states.json.gz` |
| 数值/外物适用性 | 逐块raw顶点对自身compiled凸包最大超出：link5 **9.615nm**、法兰 **2.233nm**；306块均有点超过原1e-10m，附查**FAIL**。不是全源面/全union覆盖结论；旧严格覆盖FAIL不变。完整外物适用性 **UNKNOWN** | `compiled-numerics.json`, `inherited-risks.json` |
| 动力学/正式交付 | **UNKNOWN / 未运行、未批准**；原1401步控制FAIL、7000步全幅FAIL、native/strict=false、抓持未验证均保留 | `api-audit.json`及原始失败目录 |

重点后态：1314原/新均无腕部接触；1315、1321、1326、1400、1401原最小contact.dist分别约−0.001708、−0.019974、−0.022835、−0.004027、−0.004022mm，新均无。双方都用保存post-qpos作新FK和静态碰撞；RK4最后子阶段缓存另附，未混作相同时间对齐或重新计算接触力。921个相对变换与旧调查逐值一致。覆盖离散采样，不证明两样本之间的连续分离。

**未解除的风险：**125个Gripper近边界接触在J6∈[0.881094,0.95]，本921路径J6为负，不能外推完整DH限位域安全；303点最大0.299889mm根部外扩未获DH豁免；法兰103块的已采样2.170512mm外填位于原part_0范围，DH法兰局部点约(−6.503,21.213,−35.990)mm。本1401状态只覆盖approach，无法验证下降/闭合/抬升时该区域与手、碗、桌的接触。没有继承Gripper判据、kp80或其22,382项验收成绩，也未重跑该调查。

**唯一下一步：**审查这一冻结候选的数值FAIL和外填适用边界，决定是否允许进入一次限定白碗路径动力学验证；当前不自动细分、调参、改过滤或正式换模。

候选：`../../candidates/wrist_reuse/model.xml`，SHA256 `9604a4a6935a34496f93e223219f687b4df8b7aa83037dc208fc9f3499eb05c9`。环境Python3.11.16/MuJoCo3.2.3/NumPy1.26.4/SciPy1.15.3；LIBERO资源f78abd68ee283de9f9be3c8f7e2a9ad60246e95c（仅读取资产，不创建环境）。静态扫描约6.5s，保存态双模型回归约5.8s，非运行时性能基准。7项针对性测试通过；3723次FK、2802次静态collision；零物理步、forward、约束求解、Reset及禁用API调用。资源映射明确，候选自身不依赖Gripper目录或原ZIP。

仓库根目录复核（使用既有固定环境；输出目录必须不存在，LIBERO资源需显式指定）：

```bash
python -B -m unittest discover -s tests -p test_wrist_reuse.py -v
python -B -m unittest discover -s tests -p test_link5_flange_fullscan.py -v
python -B examples/right_hand/wrist_reuse.py scan --output runs/wrist-reuse-review \
  --libero-assets /path/to/LIBERO/libero/libero/assets
sha256sum -c examples/right_hand/evidence/d3_wrist_reuse/SHA256SUMS
```

`wrist_reuse.py build --zip <上述冻结ZIP> --inventory <原candidate-build.json>`只用于重新核对提取到全新候选目录；仓库已经包含网格，正常加载/静态复核不需要该ZIP或Gripper工作区。正式左右手、共享资源、旧候选/FAIL、main及用户README未修改，PR保持Draft。
