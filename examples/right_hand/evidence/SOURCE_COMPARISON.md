# DH116 左右源定义对照

本证据比较固定上游版本 `14275cfe32cce1fe13a16df712e8a0e610a0227e` 的左右手 URDF/MJCF，并说明本仓右手模型采用的定义。逐 link/joint 字段、全部原始 STL SHA256、源文件 SHA256、编译对照与装配测量见 [source-comparison.json](source-comparison.json)。动力学和接触验收另行记录。

| 源文件 | SHA256 |
|---|---|
| 左 URDF | `5e8d24141b39a2205cd90933383e0f06f32cd9333ddb48c015b677b1ec938c1e` |
| 左 MJCF | `df202ed6244d2c7323c25695a9538b605cb8a15f9024f87c6b24f795d630f15d` |
| 右 URDF | `c3ec0a6bd0e39477554e8a1a04b7a7e44004737a5a65f86b1c02bc4848911525` |
| 右 MJCF | `a757fdd0d3c23c274e201426acc516ce107a6df63b7f9d5a1f780eb8a0a9021c` |

两侧各有 17 links、16 joints：11 个运动关节及 5 个固定关节，其中 6 个主动、5 个从动。主动顺序均为 `finger11, finger12, finger21, finger31, finger41, finger51`。两侧各自 URDF 与 MJCF 的机械字段对照无差异；其 RPY 与 `eulerseq="XYZ"` 表达相同的固定轴旋转。右手质量总和为 `0.23767046 kg`，左手为 `0.24564069 kg`。

左右手具有实际源差异，不能通过简单镜像左手定义生成右手。例如，左 thumb 基座原点为 `(0.018, -0.021464, 0.040041)`，右为 `(0.018, 0.021747, 0.043279)`；`finger11` 轴分别为 `(0,0,1)` 和 `(0,0,-1)`；`finger12` 的局部 z 原点分别为 `-0.0023593` 和 `-0.0056093 m`。JSON 的 `left_right_mechanical_field_differences` 逐项保留全部机械差异。

## 右手采用规则

机械定义完整来自右 URDF：质量、惯性原点及六分量完整惯性、装配原点、父子关系、关节类型、轴、限位和线性 mimic。URDF RPY 按 `Rz(yaw) Ry(pitch) Rx(roll)` 转换为 MuJoCo `wxyz` quaternion。5 个固定件不产生自由度。

| 从动关节 | 驱动关节 | 采用的线性关系（rad） | 上游 MJCF 的额外二次系数 |
|---|---|---|---:|
| `finger13` | `finger12` | `0.004592 + 0.8808*q` | `0.469825*q²` |
| `finger22` | `finger21` | `0.011072 + 0.9206*q` | `0.200535*q²` |
| `finger32` | `finger31` | `0.011072 + 0.9206*q` | `0.200535*q²` |
| `finger42` | `finger41` | `0.011072 + 0.9206*q` | `0.200535*q²` |
| `finger52` | `finger51` | `0.011072 + 0.9206*q` | `0.200535*q²` |

采用 URDF 线性定义以保持已验收左手接口的一致性；源从动限位仍原样保留。满驱动处，源 MJCF 二次关系比线性关系多弯拇指 `7.38°`、其余手指约 `22.40°`。本模型未采用这些二次项。

以下仿真参数另行明确取自同版本右 MJCF：运动关节 `damping=2`、`armature=0.01`、`frictionloss=0`；6 个 position actuator 的 `kp=50`、`kv=2`，目标范围来自右 URDF 主动限位；equality 的 `solref="0.004 1"`、`solimp="0.9 0.99 0.001 0.5 2"`；手接触的 `condim=4`、`friction="1 0.005 0.001"`。组合模型的 options、NERO、法兰、mount 和 7 个 arm motors 保留现有左组合字段。**这属于 URDF 机械/线性定义加单独注明的 MJCF 仿真参数，不表示与上游 MJCF 整体等价。**

## 网格、碰撞与装配

左右各 17 个原始 mesh 均为 binary STL。STL 本身不携带长度单位；两种源模型均不设置 mesh scale，直接将其数值坐标按米使用。右 base 网格尺寸约为 `62.31 × 83.42 × 108.35 mm`，交付 STL 保持源字节。

初始碰撞配置命名为 `coarse17`：全部 17 个右 URDF per-link collision mesh 启用 `contype=conaffinity=1`，由 MuJoCo 取凸包；未使用左手代理、未添加接触排除。上游 MJCF 默认关闭手接触，仅在 distal 与固定指尖 mesh 打开接触；本配置明确采用 URDF 的全部碰撞表示。整体凸包不能复现凹陷三角表面，源网格一致性不代表已通过物理接触验收。

右 hand base 在 `robot0_gripper_flange` 下保持 identity。左右 base 在 z≈0 处都存在以原点轴为中心、半径约 `11/15/24 mm` 的环形顶点；右侧分别检测到 `22/58/41` 个，左侧为 `22/55/41` 个。这支持共用原点与底面约定；不构成螺栓公差或精确凹面干涉配合认证。世界坐标与完整测量保存在 JSON。

## 验证边界及源观察

MuJoCo `3.2.3` 下，右交付模型与源右 MJCF 的 17 个 link 在 `mass/ipos/principal inertia/iquat/local pos/local quat` 上均逐值严格相等。生成 XML 的完整惯性六分量逐值等于右 URDF。完整惯性经过 MuJoCo 特征分解再重建时，两模型均观测到最大绝对残差 `1.6612e-11`；`2e-11` 阈值仅用于该数值诊断，不用于证明物理等价。

源 URDF 的运动关节 `effort=0, velocity=0` 不提供已验证的真实执行器能力，不被施加为零驱动力限制。固定关节仍有冗余 axis/limit，但不生成 DOF。左 `finger33_link` 的源 COM z 为 `-0.037473 m`，落在其网格 z 范围 `[0.015874, 0.053527] m` 之外；右手未出现该异常，不从左手复制该值。

[固定版本上游仓库](https://github.com/sorrowfeng/leadtron_hand_descriptions/tree/14275cfe32cce1fe13a16df712e8a0e610a0227e/DH116/DH116_URDF_MJCF_Files) · [MuJoCo mesh 参考](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh)
