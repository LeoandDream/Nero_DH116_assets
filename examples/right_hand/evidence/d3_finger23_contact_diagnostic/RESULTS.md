# finger23 / saved RK4 4768–4769

**结论：厂家源面真实交叉；不是只有凸包外填。下一步优先复审科研早停指标，不削减 finger23。** 基线 `f0b37997e38ed34c46a3dd7f223168eff2d3126b`，[任务](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6290843240)。仅分析两步原始 RK4 末内部阶段缓存，另核对相邻 g22；未用 post-step FK 代替。

| 保存步 / 原生碗壁 | 厂家源面最小表面距离 / 严格交叉 | 原生 MPR 深度 (mm) | 法向力 (N) | compiled 单对分离平移的认证上界 (mm) |
| --- | --- | ---: | ---: | ---: |
| 4768 / g22 | 0 / 是 | 1.946405394 | 1.625914576 | 1.857883192 |
| 4768 / g23 | 0 / 是 | 1.929396020 | 1.701058596 | 1.758616013 |
| 4769 / g22 | 0 / 是 | 1.966277183 | 1.634894440 | 1.877371925 |
| 4769 / g23 | 0 / 是 | 2.029397981 | 1.737957894 | 1.777561496 |

**确认与边界。** 厂家右手 STL（5876 面，SHA `ccbd2de1…`）与交付 raw STL（5874 面，`730786ab…`）不是同一文件，分别检查后均与完整原生 box 交叉。厂家 g23 见证为源面 **5327**；4769 的 link-local 交点为 `[4.826256, 1.308449, 18.461747] mm`。实际 MPR 点附近的最近厂家源面点也在 g23 内，两步至最近 box 平面净距分别为 **0.426491 / 0.434564 mm**；这些是局部边界点净距，不能称为材料穿透深度。源 STL 边拓扑水密且一致定向，但没有全局自交或材料形变证书。

厂家、交付 raw、raw `_06`、compiled `_06` 共 **16 项**连续三角面查询均有严格交叉见证，以 60 位 Decimal 和精确有理数独立复核。完整凸体/box 交集在 g23 有 **14 个**候选代理边界 patch（g22 为21）；不能由 MPR 中点/法向指定唯一厂家受力面。raw 与 compiled 凸体的顶点配对 Hausdorff 上界估计约 **0.862 nm**，不是 0.1 nm 数值验收豁免。[局部截面](local-section.png)中的黑叉只是 MPR 点投影。

**为何先审查判据。** g23 原生深度增加 **100.002 µm**，法向转动 **7.151268°**；完整 box 在 finger frame 中最大角点位移仅 **33.220 µm**。更直接的证据是：对全部86个实际 mesh-graph 支撑点与完整8角点，精确支持不等式证明4769存在 **≤1.777562 mm** 的分离平移。因此 **2.029398 mm 不是这对凸体的精确欧氏 MTD**。上界不是 MTD 精确值，也不代表整手可达、其他接触同时消失或可接受材料形变。证书只针对保存几何及声明的变换舍入界，不承诺任意模型扰动下仍成立。

MuJoCo 3.2.3 默认 flags=0 使用 [libccd MPR 的深度及中点](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_collision_convex.c#L297-L317)。实际混合 `condim=4`、`solref=[0.0105,1]`、`solimp=[0.949,0.974,0.001,0.5,2]`、摩擦 `[1,1,0.3,0.1,0.1]`；不是碗单侧 `solref=0.001`。[参数混合源码](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_collision_driver.c#L1159-L1241)、[软接触说明](https://mujoco.readthedocs.io/en/3.2.3/modeling.html#solver-parameters)。约1.74N及两帧不足以证明安全，也不足以授权任意增大阈值。

**唯一建议：** 先审阅并预注册科研接触停止指标，区分原生 MPR 深度、独立凸体分离证据、持续时间/力及控制健康。本轮不提出新数值、不改2mm、不修网格、不申请自动重跑。旧4769硬早停 **FAIL** 不回写；完整抓持 **UNKNOWN**、正式高保真/原0.1nm失败结论均保留。

## 执行偏差与可复核文件

**零新 `mj_step` 的任务约束未完全满足，必须单独记为 FAIL。** 导出 compiled 顶点的三个原型分别成功编译一次模型；Python API 守卫未拦截编译器内部调用。固定3.2.3源码表明每次编译会禁用 CONTACT、执行内部 Reset/FK 并调用一次 `mj_step`，因此推定发生 **3次编译自检积分**（未原生插桩）。没有重跑白碗、控制或保存缓存轨迹；发现后停止编译，改用纯离线入口。[编译源码](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/user/user_model.cc#L3736-L3791)。完整过程、原型 SHA 及无效 Python 计数的更正在 [extraction-disclosure.json](extraction-disclosure.json)，不能将旧计数0解释为原生零调用。

- [geometry.json.gz](geometry.json.gz)：原始触点/法向/六维力、完整几何、严格交叉/分离证书、源路径和 SHA。
- [diagnose.py](diagnose.py)：**不导入 MuJoCo，不编译模型**，以已有 RK4 缓存和导出顶点复核16项交叉、4项凸体交集/分离证书及局部源点。
- [verification.json](verification.json)：上述纯离线复核通过；compiled 86支撑点 SHA `b9a56255…`、168面索引 SHA `20fbaa98…` 与既有 `d3_finger22_contact_regions/evidence.json.gz` 完全一致，追溯实际编译形状。未逐一重新导出全部431资产顶点。
- 复核1206个原跟踪文件均未变；用户 README SHA仍为 `d68ec6d6…`。没有模型、动作、控制、过滤、阈值或历史结果修改；本目录仅保存诊断证据。

运行环境 Python3.11.16 / NumPy1.26.4 / SciPy1.15.3，图使用已有 Matplotlib。仓库根目录执行（输出目录须不存在）：

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MPLCONFIGDIR=/tmp/finger23-mpl \
  python -B examples/right_hand/evidence/d3_finger23_contact_diagnostic/diagnose.py \
  --output runs/finger23_offline_recheck
```

这里的 `OFFLINE_GEOMETRY_RECHECK_PASS` 仅指保存数据的几何复核；不代表动力学、抓持、正式资产或本轮零积分约束全部通过。
