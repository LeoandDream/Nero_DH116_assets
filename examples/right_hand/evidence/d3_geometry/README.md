# D3：右手碰撞失败几何归因

2026-10-18；依据[任务评论](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6249008825)。输入固定为`ce920568e159ac38af4fda412ac84f2262e27035`中的7000步轨迹与右手原始STL；模型SHA仍为`739d3d29bbfd4a6468fb8b75f4de09b719c0f96abb39508959c1dfc33a84b9cc`。本目录仅新增诊断代码/结果，全部既有文件保持原样。

**结论：当前碰撞筛查仍FAIL。“两组失败仅由整块凸包填凹造成”被原始三角面横穿证据否定。二次Mimic会改变后期第二组几何关系，但不能消除第一组干涉，也未获得新的动力学PASS。**

## 事件—几何—证据

`A=base_link↔finger12_link`，`B=finger12_link↔finger42_link`。负接触距与力来自原始RK4记录；原三角面结果来自对应记录行的post-step FK。交叉用连续边穿面证书确认，没有以最近顶点代替表面距离。

| 步／阶段 | A原接触距mm／法向力N | B原接触距mm／法向力N | 原始STL连续表面结果 |
|---|---:|---:|---|
| 229／拇指转动上升段 | −2.0122／37.867 | 无 | A严格横穿；B间距89.616463mm |
| 451／拇指保持，首次稳态超限 | −6.0323／62.146 | 无 | A严格横穿；B间距84.726625mm |
| 6498／联合保持，法向力峰附近 | −5.5633／57.342 | −12.4204／66.637 | A、B均严格横穿 |
| 6511／联合释放，交叠峰 | −5.5483／55.877 | −12.4776／64.963 | A、B均严格横穿 |

全部记录状态、18关节值、目标、误差、接触位置/法向见[selected_trace.json](trace/selected_trace.json)；全部body/geom世界变换见[checkpoint_poses.json](trace/checkpoint_poses.json)。例如229步A接触点为`(-0.0909182325,0.2371643075,1.1320733581)m`，法向为`(0.7072618072,0.6077750718,-0.3610958297)`。

交叉证书保留零起始原STL面号、三角顶点、交点、两端平面异号距离、面内重心坐标、双body局部坐标及法向：[surface_results.json](triangle/surface_results.json)。例如229 post的A见证面号为`93822/5275`，6511 post的B为`6776/6761`。全部实际横穿见证距相应记录接触点不超过5.465mm；229/451的A及6498/6511的B在对应pre行也横穿，支持这些附近姿态的稳健性。**证实源表面相交，不等于证实原STL有12.48mm实体穿透深度。**

[四个源Mesh关键帧](figures/keyframes.png) · [原表面断面与凸包对照](figures/contact-sections.png) · [Mimic离线断面对照](figures/mimic-sections.png)。图片为辅助定位，单一断面或渲染不代替全表面检查。

## 相同主动角下的Mimic反事实

保持机械臂及6主动关节记录角度，仅在离线查询数据中分别设置5从动为精确URDF线性公式或源MJCF二次公式；不修改正式Equality、不解约束、不写入动态轨迹。

| post步／B | 记录状态 | 精确线性 | 二次公式 |
|---|---|---|---|
| 6498原STL | 相交 | 相交 | 表面间距 **+0.598545mm** |
| 6511原STL | 相交 | 相交 | 表面间距 **+0.729187mm** |
| 6498凸包零cutoff查询/mm | −12.4204 | −12.3800 | −0.7954 |
| 6511凸包零cutoff查询/mm | −12.4775 | −12.3869 | −0.6708 |

B只受5个从动中的`finger42`影响；相对记录状态，二次公式使其在6498/6511增加约`0.282584/0.283408rad`。A的祖先链没有从动关节，三模式变换严格相同，因此A的源表面交叉不变。[40次单从动扰动核对](trace/passive_independence.json)。这是固定姿态下的几何独立性，不能推出改变Mimic对未来动力学毫无影响。

二次反事实下，B的表面分离而凸包仍重叠，**支持该反事实姿态存在凸包额外接触**。三份原STL均非严格水密2-manifold，不能由正表面距离直接认证实体完全无重叠；该实体判断仍为UNKNOWN。此反事实也不能证明哪套Mimic更符合硬件或其动态跟踪会通过。

## 数值与时序边界

- 方法：AABB BVH下界剪枝，连续顶点—面、边—边及边穿面距离。正距时穷尽有效候选；严格横穿即全局表面最小距0。float32源顶点提升至float64，原面号保留。[源SHA、拓扑与算法精度](triangle/metadata.json)。
- 12条直接查询的横穿见证（含精确线性反事实）经60位Decimal独立重算；4个正距最近面均由重心坐标凸QP核验成功。8个解析案例及899面对穷举/BVH对照通过：[数值复核](triangle/focused_verification.json)。这是有限精度几何证据，不是精确谓词或硬件认证。
- RK4记录的接触缓存来自最后内部stage，而qpos是最终加权状态；pre/post均不等同精确接触缓存姿态。229步A的post重算接触距与原记录差0.181μm，pre差67.12μm。[时序残差](trace/alignment_audit.json)、[MuJoCo 3.2.3源码](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_forward.c#L751-L846)。
- `mj_geomDistance`在这些Mesh上对`distmax`敏感，1m cutoff甚至会反转反事实比较方向。JSON保留0/.01/.1/1m查询；上表采用0-cutoff负距离，并与离线`mj_collision`核对。零值只表示cutoff截断，不证明正间距；全表面结论依据独立三角算法。不要用旧`distance_m`（1m查询）解释深度。

## 唯一优先假设与下一组实验（待确认，未执行）

**H：仅在首个拇指组协调主动`finger12`与`finger11`的命令，可以避开首次A干涉。** 当前为UNKNOWN，不能说“finger12保持打开已被证明是全部失败主因”：6498/6511的A仍相交，此时finger12实际约0.43rad；此实验也不处理B。

只改变首组1000步（2s）内`finger12`的目标：`q12_target(t)=λ·0.523599·f11(t)`，其中`f11(t)`为现有finger11的归一化开合相位；对照`λ=0`复用现有前1000步，候选`λ=1`需确认后单独执行。其他目标、原生Reset完整初态、JPC初始化、线性Mimic、源Mesh、物理参数及原阈值均不变，不加新控制器。

预期证据：同时记录实际q11/q12、A接触、稳态误差与原三角面交叉。若实际q12能跟踪且A交叉/超限消失，支持首组路径修复；若仍交叉或超限，该候选FAIL。若q12未跟随，则命令修复候选失败，几何路径可行性仍UNKNOWN。即使局部改善，也不替代原7000步验收，不消除后段A/B的阻塞。**相同姿态下，忠实凸分块不能消除已证实的源表面相交，当前不优先分块任何link或替换Mimic。**

## 离线复算

```bash
source examples/libero_reference/env/activate.sh
python -B examples/right_hand/evidence/d3_geometry/reproduce.py --output runs/d3_recheck_001 --figures
```

输出必须为新目录；依赖复用既有锁定环境。已实际执行该入口；步进、积分与约束求解API被禁用，新增物理步/约束求解均为0，全部输入及原有tracked文件哈希不变：[只读核验](read-only-check.json)。本目录文件另由`SHA256SUMS`绑定。当前全幅筛查仍FAIL，抓持UNKNOWN；修改模型或执行上述物理对照须先确认。
