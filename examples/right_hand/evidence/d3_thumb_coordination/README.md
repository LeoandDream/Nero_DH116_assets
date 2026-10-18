# D3：1000步拇指同相协调对照

2026-10-18，依据[执行授权](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6249954666)。固定源基线`8142d0ca9a6e78332118ab5e632cafe4e39da28b`；执行入口提交`30216a660974fbfcd8344cbc7a734f5ab816a790`。

**结论：λ=1候选FAIL，未支持“本次同相协调可消除首组A干涉”的局部假设。finger12闭合稳态跟踪满足原0.02rad界限，但A的原始三角面仍横穿、报告交叠仍超过2mm，finger11稳态跟踪仍超限。原7000步全幅开合仍FAIL，抓持UNKNOWN，LIBERO未运行。**

## 唯一干预与身份

λ=0复用原保存轨迹/命令的前1000步；本轮只新增一次λ=1、1000个真实`mj_step`（2s）。仅将finger12目标改为`0.523599*f11(t)`rad；finger11及其余目标、250步×4阶段、原生Reset/JPC初始化、模型/Mesh/线性Mimic、控制与物理参数及全部原判据保持不变。实际13D动作逐行核对，只有索引8改变，共749个非零差值；其他目标/相位/记录时间严格相同。[实际干预核对](intervention-audit.json)。后续机械臂力矩由相同JPC反馈生成，允许因物理响应而变化。

[基线审计](baseline-audit.json)确认原7000条动作与生成器相同，保存文件压缩/解压SHA一致，首1000条动作/目标/ctrl对齐；Python可执行文件、58个依赖版本及MuJoCo/NumPy/stock robosuite等wheel文件身份一致，无须额外重跑λ=0。原实验未保存完整`mjSTATE_INTEGRATION`快照；初态/缓存对齐依据同引擎、同模型及相同初始化源码恢复，并非原integration快照直接字节比对。

实际在独立克隆`/data_robot/verification/nero_dh116_right`执行；bubblewrap隐藏`/data_robot/projects`及旧`/tmp`。[隔离核验](isolation-exit.json)。[Reset与独立计数](reset-and-step-audit.json)记录18D初态、零qvel/ctrl/time、5组启用Equality、原顺序`reset→forward→JPC内两次forward→set_goal→首step`，独立调用/完成数均为1000。[环境身份](environment.json)、[全部输入SHA](input-identity.json)。

## λ=0 vs λ=1

`A=base_link↔finger12_link`。时间为记录时间；“交叠”是MuJoCo凸包接触报告值，不是原STL实体穿透深度。

| 指标 | λ=0原记录 | λ=1唯一新运行 |
|---|---:|---:|
| 真实步数／状态与接触行数 | 原1000／各1000 | 新1000／各1000 |
| 非有限状态／MuJoCo数值warning步数 | 0／0 | 0／0 |
| q11闭合稳态最大误差/rad（界限0.02） | **0.02406233 FAIL** | **0.02272393 FAIL** |
| q12闭合稳态最大误差/rad | 0.00007946（目标0） | **0.00449262 PASS** |
| q11／q12全阶段误差峰/rad（含非稳态） | 0.179022／0.000364 | 0.177931／0.114567 |
| 首次A报告接触 | 200步／0.400s | 201步／0.402s |
| 首次A交叠超2mm／首次数值筛查失败 | **229步／0.458s** | **231步／0.462s** |
| A报告接触步数／超2mm步数 | 428／368 | 426／365 |
| A最小报告接触距/mm | −6.047436 | −5.915937 |
| 最大启用非相邻报告交叠/mm | 6.047436 | 5.915937 |
| A峰值法向力/N | 62.233167（500步） | 60.881681（500步） |
| 首次稳态跟踪超限 | 451步／0.902s | 451步／0.902s |
| 原始STL严格横穿帧数 | **428／1000** | **426／1000** |
| 首个／末个post状态横穿 | 200–627步／0.400–1.254s | 201–626步／0.402–1.252s |

首次报告交叠超限只延后2步（4ms），源表面首次横穿只延后1步（2ms）。交叠/受力略降不构成消除干涉。原闭合/释放稳态窗口及阈值未变，不能把全阶段瞬态峰当作稳态判据。[结构化结果](result.json)、[原始筛查结果](lambda1-result.json)、[对比图](comparison.png)。

## 源三角面证据

复用既有连续三角面算法，对两条实际记录的全部2000个post qpos做FK；缓存的相交面仍逐帧重验，未命中时完整穷尽重叠BVH候选。分离帧需要排除全部候选；没有以顶点采样或MuJoCo无接触代替原表面判断。λ=0/1分别572/574帧表面分离，两组边界/数值未定帧均为0。[逐步几何压缩记录](geometry/all_steps.jsonl.gz)、[汇总](geometry/summary.json)。

双方共享的229、231、451、500步均为严格横穿。例如λ=1第451步实际`q11=1.02447407`、`q12=0.51910638`rad（目标`1.047198/0.523599`）；q12已充分进入稳态界限，A仍横穿：源面号`base=95972 / finger12=5330`，交点world约`(-0.0888711064,0.2313018137,1.1287091393)m`。全部854条横穿见证经原60位Decimal算法独立复核，最大交点差`6.94e-18m`；另对共享关键帧做完整连续表面查询。[见证和复核](geometry/supplemental_verification.json)、[关键帧](geometry/keyframes.json)。

接触/力缓存仍来自RK4最后内部stage，FK使用最终加权post qpos；同一行的两者是附近而非完全相同姿态，不做精确等同。源STL非严格水密2-manifold，表面分离不认证实体完全无重叠；这些限制不影响已记录严格横穿对当前候选的否定。离线几何部分新增物理步及约束求解均为0。

## 唯一后续建议与复现

**只围绕A的已定位铰接区域，核对厂家源右手的局部装配几何及碰撞语义，形成该对接触的局部修复提案后再确认实施。** 当前证据不支持继续扫描协调系数、调伺服或盲目分块；相同姿态下，忠实分块不能消除已证实的源三角面相交。该装配交叠是否符合源设计意图仍UNKNOWN，本轮没有修改任何接触定义或正式模型。

```bash
source examples/libero_reference/env/activate.sh
# 下列第一条会执行一次新的1000步，仅用于有意复现；本轮已执行一次，未重跑。
python -B examples/right_hand/thumb_coordination.py --output runs/d3_thumb_lambda1_001
# 以下只分析已记录数据，不增加物理步：
python -B examples/right_hand/thumb_geometry.py --run-output runs/d3_thumb_lambda1_001
```

实际第一条退出码`2`，含义是原数值筛查FAIL，完整1000步仍已记录；离线分析退出0。[真实命令/退出码](execution.json)。3项纯计划测试通过，断言只有一个目标通道变化；没有执行额外动力学测试。补充见证复核入口为`verify_thumb_geometry.py --run-output <原运行目录>`。

压缩状态/接触/动作及逐步几何的压缩和解压SHA、行数在[artifact-index.json](artifact-index.json)，本目录文件由`SHA256SUMS`绑定；原始λ=0数据仍引用上级已有文件，未复制或改写。全部原378个tracked文件、左手V1、原main工作树及未提交README保持原样：[保护核验](protection.json)。新材料仅提交当前独立Draft PR分支，不合并main。
