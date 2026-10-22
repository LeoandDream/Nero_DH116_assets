> **版本说明（Issue #6）**：下文描述的是早期 `mjcf/nero_dh116_right.xml` 的17源Mesh粗模型，其全幅FAIL与抓持UNKNOWN保持原结论。新的研究交付入口为 [`mjcf/dh116_right.xml`](../mjcf/dh116_right.xml) 和 [`mjcf/nero_dh116_right_research.xml`](../mjcf/nero_dh116_right_research.xml)，继承 `7998718` 的81手部＋306腕部、B驱动配置；固定白碗功能结果见[原始成功证据](../examples/libero_eval/evidence/functional_v1/RESULTS.md)，交付对应性见[四模型验收](../examples/evidence/four_assets/RESULTS.md)。不能混用两版验证结论。

# NERO＋DH116-R000-A1 右手组合

独立入口为 [`mjcf/nero_dh116_right.xml`](../mjcf/nero_dh116_right.xml)，源描述为 [`urdf/dh116_right.urdf`](../urdf/dh116_right.urdf)。左手入口、Mesh、控制参数、D1/D2测试和验收证据保留原样。左右手具有相同维度的接口，但机械与碰撞模型不同，左手白碗结果不能作为右手验收成绩。

## 固定来源与定义选择

厂家描述仓库固定为 `sorrowfeng/leadtron_hand_descriptions@14275cfe32cce1fe13a16df712e8a0e610a0227e`，右手目录为 `DH116/DH116_URDF_MJCF_Files/DH116-R000-A1/`。[源清单](../RIGHT_SOURCE_MANIFEST.json)绑定每份源Mesh、URDF/MJCF和交付文件的SHA。

- 刚体、完整惯性、16个关节的装配、11个旋转轴和限位均从右URDF读取；5个固定关节保留刚体装配，不额外产生DOF。URDF RPY按`Rz(yaw) @ Ry(pitch) @ Rx(roll)`转为MuJoCo quaternion。
- 5组Mimic采用右URDF的一阶线性定义：拇指`q13=0.004592+0.8808*q12`，其他4指`qFollower=0.011072+0.9206*qDriver`。这与左手定义来源一致。
- 上游右MJCF另含二次项：拇指`+0.469825*q12²`、其他4指`+0.200535*qDriver²`。本模型未使用这些项，也不声称与该MJCF整体动态等价。主动关节到源上限时，线性/二次结果分别相差约7.38°和22.40°。
- 仿真参数单独采用固定右MJCF所声明的`damping=2`、`armature=0.01`、`frictionloss=0`、6个position servo的`kp=50, kv=2`、接触`condim=4, friction=(1,0.005,0.001)`，以及其Equality求解参数。它们不是URDF测得的驱动规格。源URDF的effort/velocity为0，不能据此证明真实硬件力矩或速度上限。
- NERO、法兰、挂载座、7个电机、原有base/link1接触排除和模型级求解参数从已验收左手组合原样保留。

| 关键源差异 | 左手 | 右手 |
| --- | --- | --- |
| 型号 | DH116-L000-A1 | DH116-R000-A1 |
| `finger11`轴 | `(0,0,1)` | `(0,0,-1)` |
| `finger11`装配xyz/m | `(0.018,-0.021464,0.040041)` | `(0.018,0.021747,0.043279)` |
| 基座质量/kg | 0.11674 | 0.10866 |
| 手总质量/kg | 0.24564069 | 0.23767046 |

右模型直接来自右源数据；没有通过镜像左手构造机械定义。

## 装配与碰撞范围

法兰到右手base采用局部identity。几何依据是两侧厂家base的底面均在局部`z≈0`、共享以原点为中心的11/15/24mm底面圆环，且相同法兰接口坐标可对照。这是仿真装配约定，不代替定位孔、公差或实物安装测量。`grip_site`、`ft_frame`等接口site在手base原点，不代表指尖或抓取中心。

右手保留17份原始STL并全部作为碰撞Mesh启用；MuJoCo对每份Mesh使用凸包。它是明确的粗碰撞表示，没有复用左手76块代理，也没有通过关闭整手碰撞获取结果。上游MJCF只开启部分远端碰撞的配置未整体导入。凸包会填充槽孔、桥接凹部；全闭姿态的相交候选及默认过滤对需要单独记录，不据此宣称精确三角面碰撞或真实材料穿透。

## 接口与最小运行

[右手接口契约](robot-contract-right.json)：18D关节状态；13D动作的前7维为NERO绝对位置、后6维为独立归一化手目标。原生JPC把机械臂位置目标变为7D力矩；position servo接收6D主动关节角。Robot action不能直接写入`data.ctrl`。

依赖复用现有固定运行环境，Python3.11.16、MuJoCo3.2.3、stock robosuite1.4.1及完整锁文件均保持原样：

```bash
git clone --branch feature/dh116-right-v1 https://github.com/LeoandDream/Nero_DH116_assets.git
cd Nero_DH116_assets
bash examples/libero_reference/env/bootstrap.sh
source examples/libero_reference/env/activate.sh
python examples/right_hand/run.py --output runs/right_hand_001
python -m unittest discover -s tests -p 'test_right_*.py' -v
```

脚本直接加载正式右组合；以stock JPC保持NERO位置，执行6个通道分别开合及联合开合，记录真实`mj_step`、Mimic、限位、接触、交叠与数值警告。外物接触探针是独立简单场景，不构成抓持测试。运行不需要上游clone、历史实验目录或私有仓库。

协议在首次完整运行前固定：单通道及联合开合共7000步（14s）；独立外物探针2000步（4s），`dt=0.002s`。探针仅在Reset前按几何选择无初始交叠的位置，执行中由重力与接触演化，不注入状态。筛查界限为：机械臂保持误差0.1rad；手部稳态跟踪、Mimic残差各0.02rad；关节越限0.01rad；启用的非相邻自接触及外物交叠各2mm。角度界限是本轮最低诊断门槛，2mm沿用现有接触诊断口径，均不是厂家精度/安全认证。失败不会自动调参或中途关闭碰撞。

退出码`0`表示最低筛查全部通过，`1`表示执行错误，`2`表示完整记录中的FAIL或UNKNOWN。原始输出保存在指定的新目录，包含逐步状态/接触JSONL、冻结动作、参数、源哈希、静态距离检查和结果；重复运行需选择新目录。

如需复核转换，可单独克隆固定厂家提交，再执行`python tools/build_right.py --upstream <source-checkout> --output-root <new-directory>`；运行模型和默认测试不需要该源码缓存。模型生成只使用固定源描述和原始Mesh，不生成分块代理。

实际结果见[右手验收记录](../examples/right_hand/evidence/RESULTS.md)：静态机械检查通过；9000步完整执行且无数值警告，基础掌部外物接触通过最低筛查，但全幅开合的自接触交叠和稳态跟踪未通过。此版本作为可复核的右手候选提交，不代表右手动力学整体验收通过。右手抓持能力保持UNKNOWN，未运行LIBERO完整任务。

右手源文件与左手来自同一固定厂家仓库，沿用其[Apache-2.0许可](../licenses/DH116_APACHE_2_0.txt)，来源对照见[源证据](../examples/right_hand/evidence/SOURCE_COMPARISON.md)。法兰许可风险维持[现有披露](../licenses/SOURCES.md)。
