# 模型与接入说明

## 组成与坐标

组合模型入口为 [`mjcf/nero_dh116.xml`](../mjcf/nero_dh116.xml)，静态展示入口为 [`examples/scene.xml`](../examples/scene.xml)。机器人由 NERO 七轴机械臂、厂家法兰和 DH116-L000-A1 左手组成。

| 项目 | 约定 |
| --- | --- |
| 单位 | 长度 m、角度 rad、质量 kg、机械臂力矩 N·m |
| 基座 | `robot0_base_link`，固定到世界，位置约 `(-0.569, 0, 0.912)`，四元数 `(1, 0, 0, 0)` |
| 参考姿态 | MJCF 的第一个 keyframe；使用 `mj_resetDataKeyframe` 后执行 `mj_forward` |
| 手部相机 | `robot0_eye_in_hand`；外参属于仿真场景配置 |
| 挂载座 | `mount0_` 前缀的模型几何；查看器默认隐藏 |
| 状态 | NERO 7 个＋DH116 11 个关节位置；顺序见 [robot-contract.json](robot-contract.json) |

参考姿态和基座位置不是零位安装标准。改变机器人在场景中的安装位置时，应统一处理机器人与挂载座的相对关系，并核对相机视野。显示隐藏不删除挂载座的物理几何。

`urdf/nero.urdf`、`urdf/dh116_left.urdf` 和 `urdf/nero_plus_manufacturer_flange.urdf` 分别是机械臂、左手和机械臂带法兰的描述；组合装配由 MJCF 提供。源描述位于 `urdf/source/`。MJCF 的网格引用以模型文件位置为基准，来源清单中的原始路径用于溯源，不是运行时依赖。

## 驱动与动作语义

NERO 的 7 个 motor actuator 接受力矩；DH116 的 6 个 position actuator 接受主动关节目标角，5 组 equality joint constraint 表达源 URDF 的线性 Mimic 联动。

外部 13D Robot action 的机械臂部分是绝对关节位置 rad，需要位置控制器计算力矩；手部为 6 个独立归一化目标，按主动关节范围映射到角度。手部通道依次为 `finger11`、`finger12`、`finger21`、`finger31`、`finger41`、`finger51`。参考控制组合为 robosuite 1.4.1 `JointPositionController` 的 `set_qpos` 绝对位置入口和 MuJoCo position 伺服。

两类控制输入均为 13D，但单位和作用不同。初始化 keyframe 也不替代保持姿态所需的位置控制。加入场景物体后，应按契约中的关节名称读取 18D 机器人状态，而非假设整个 `data.qpos` 都属于机器人。

## 机械参数与求解设置

质量、惯性、关节轴、限位及线性 Mimic 以源 URDF 为依据。源文件缺失或填零的机械参数仍是未标定项；手部伺服增益和约束参数是仿真配置。NERO 的 `ctrlrange="-100 100"` 是仿真力矩输入范围，真实额定力矩需另查硬件资料。DH116 真实力矩与速度上限、法兰安装及相机外参尚未完成实机标定。

当前 MJCF 含以下模型级设置：

| 设置 | 值 |
| --- | --- |
| 验证使用的引擎 | MuJoCo 3.2.3 |
| 积分器／物理步长 | RK4／0.002 s |
| 摩擦锥／阻抗比 | `cone="elliptic"`、`impratio="20"` |
| 环境流体参数 | `density="1.2"`、`viscosity="0.00002"` |

这些 `<option>` 设置作用于整个 MuJoCo 模型，而非只作用于机器人。合并到已有环境时，需由目标场景明确决定最终求解与环境参数。物理步长与外部控制频率是两个设置。

## 碰撞代理及其影响

当前碰撞表示为分块凸网格：NERO 51 块、DH116 76 块、法兰 8 块。主要接触指尖 `finger14_link` 和 `finger23_link` 各采用 12 个凸块；它们按源三角面分区，由源顶点生成。可视网格与碰撞网格各自承担渲染和接触计算。

| 影响 | 对使用者的意义 |
| --- | --- |
| 接触位置与间隙 | 凸代理仍可能填充局部凹部，使接触早于可视表面发生。碗沿、薄物体、狭缝和插接任务应特别检查接触间隙。 |
| 接触法向与抓持 | 代理表面的法向和接触点分布会影响接触力矩、滑移和抓持姿态；视觉贴合不直接代表有效夹持。 |
| 图像与物理一致性 | 相机观察可视网格，求解器使用碰撞代理。毫米级外形差异可能影响基于图像定位的接近与闭合。 |
| 求解开销 | 分块会增加潜在几何接触对及约束数量。实际开销取决于场景、接触数量和求解设置。 |
| 惯性与几何 | 模型显式保留机械惯性。碰撞近似服务于接触计算，不替代质量与惯性标定。 |

碰撞结果还受物体侧的碰撞表示、摩擦、`solref`、`solimp`、接触 margin 和时间步长影响。`data.contact[i].dist` 描述求解器接触几何的有符号距离；负值表示代理交叠，其量级与可视网格间距、真实材料变形属于不同指标。

预览时使用 `--collision` 叠加检查代理与可视表面。针对新的精细操作任务，应同时检查机器人和物体的接触面，并结合接触对、法向力、滑移及约束残差判断适用性。原理参见 [MuJoCo 碰撞检测][collision]与 [geom 参数][geom]。

## 自碰撞与显示过滤

模型显式排除固定底座与第一连杆这一对内部接触，对应来源 SRDF 的 `Adjacent` 关系：

```xml
<contact>
  <exclude body1="robot0_base_link" body2="robot0_link1" />
</contact>
```

其余接触由 MuJoCo 默认过滤、刚性组、geom 的 `contype`／`conaffinity` 以及场景接触定义共同决定。手部与外部物体保留碰撞配置。

查看器的 geom group 开关仅控制绘制；碰撞掩码和 `<exclude>` 决定物理接触过滤。新场景遇到异常接触时，应先核对坐标、几何、过滤规则和控制目标，保留任务需要的手—物体接触。

## 适用范围

模型在参考环境下已完成机械字段核对、有限姿态／几何检查，以及白碗闭合、抬升和持续对向持握验证；详见[验证说明](VALIDATION.md)。适用于这些近似条件下的仿真接入与操作实验。全工作空间可靠性、其他物体与任务、多 seed 成功率及实机接触一致性仍需按目标用途验证。

[collision]: https://mujoco.readthedocs.io/en/3.2.3/computation/index.html#collision-detection
[geom]: https://mujoco.readthedocs.io/en/3.2.3/XMLreference.html#body-geom
