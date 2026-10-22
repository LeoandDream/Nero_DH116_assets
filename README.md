# NERO × DH116 — 四套研究级 MuJoCo 资产

提供可直接加载和控制的 DH116 左右独立手、NERO＋左右手组合。保留各侧真实机械定义，不强制统一关节轴、惯性、行程或碰撞代理。面向研究仿真使用；未做全域高保真、实物标定或官方 LIBERO Benchmark 认证。

## 四个入口

| 入口名 | MJCF | state / action | 验证范围 |
| --- | --- | --- | --- |
| `dh116_left` | [独立 DH116-L](mjcf/dh116_left.xml) | 11D / 6D | 独立加载、六主动通道有界开合、Mimic与限位健康 |
| `dh116_right` | [独立 DH116-R](mjcf/dh116_right.xml) | 11D / 6D | 同上；保留右手机械与 B 驱动配置 |
| `nero_dh116_left` | [NERO＋DH116-L V1](mjcf/nero_dh116.xml) | 18D / 13D | 原字节保留左手V1；固定白碗抬升及1.5s持握证据 |
| `nero_dh116_right` | [NERO＋DH116-R 研究候选](mjcf/nero_dh116_right_research.xml) | 18D / 13D | 对应 `7998718` 的 B＋functional_v1；固定白碗抬升及1.5s持握证据 |

[四模型索引及资源SHA](docs/assets.json)是路径、顺序、维度和来源的机器可读入口。[本轮验收与局限](examples/evidence/four_assets/RESULTS.md)区分新增实际控制测试、编译等价核验及复用的抓持证据。

**右手成功配置是81块手部＋306块腕部的 B 研究候选。** 旧的[17源Mesh右手模型](mjcf/nero_dh116_right.xml)及其全幅开合FAIL仍保留；它没有继承新候选的抓持结论。[右手版本说明](docs/RIGHT_HAND.md)。306腕代理为复现冻结配置，不表示最优碰撞架构。

## 安装与运行

Linux，Python 3.11，MuJoCo 3.2.3。独立手仅需 MuJoCo / NumPy；组合控制示例额外采用 stock robosuite 1.4.1 的关节位置控制器，不需要 LIBERO。

```bash
git clone https://github.com/LeoandDream/Nero_DH116_assets.git
cd Nero_DH116_assets
# 交付审查期间切换到本任务Draft PR分支：
git switch feature/issue6-four-assets
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r examples/requirements-control.txt
python examples/control_asset.py --asset dh116_left
python examples/control_asset.py --asset dh116_right
python examples/control_asset.py --asset nero_dh116_left
python examples/control_asset.py --asset nero_dh116_right
```

命令从各模型的参考keyframe初始化并保持目标250个真实物理步。独立手的掌部固定在局部原点；组合机器人仍使用原基座装配和世界位置。独立手移除了机械臂/法兰及其自由度，没有隐藏的浮动基座。

复制模型时同时保留其相对引用的 `meshes/`。原生加载无需实验文件：

```python
import mujoco
model = mujoco.MjModel.from_xml_path("mjcf/dh116_right.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)
```

所有四入口的资源闭包仅位于 `mjcf/` 和 `meshes/`。`tools/build_four_assets.py` 是维护者的来源打包工具，运行交付模型不需要它，也不需要原实验目录或临时 LIBERO 模型。

## 统一控制接口

两侧使用相同的关节名和通道顺序，具体数值范围从所选模型读取：

- 手部 state：`finger11, finger12, finger13, finger21, finger22, finger31, finger32, finger41, finger42, finger51, finger52`，均加 `gripper0_` 前缀，单位rad。
- 手部 action：`finger11, finger12, finger21, finger31, finger41, finger51` 六个归一化绝对位置，范围 `[-1,1]`。按各自 actuator ctrlrange 线性映射到目标角度。
- 组合 state：NERO七关节在前，手部11关节在后。组合 action：NERO七个绝对目标角度rad在前，手部六通道在后。独立手无需补齐不存在的NERO状态。
- MuJoCo `ctrl`：独立手为六个主动关节角度；组合为七个NERO电机力矩＋六个手关节角度。**不能把组合13D位置action直接赋给ctrl。**

```python
from examples.control_asset import AssetRuntime
robot = AssetRuntime("dh116_right")  # 也可选择其他三个入口
command = robot.home_action()
command[-6:] += 0.1                 # 小幅增加归一化手目标
robot.set_action(command)
robot.step(25)                      # 2ms物理步；25步对应20Hz命令
print(robot.state())                # 独立手11D；组合18D
```

组合示例复用 stock `JointPositionController.set_goal(..., set_qpos=...)`，再将力矩写入原电机；手部使用原 position servo 和线性 Mimic。独立手显式保留五组掌部—近端指节的原有父子过滤，防止固定到world后MuJoCo改变这些接触资格；其余自碰撞及外物接触规则保留，详见验收记录。

## 验收与白碗实验

```bash
python -m unittest discover -s tests -p test_four_assets.py -v
python examples/check_assets.py --output runs/four_assets_001
sha256sum -c SHA256SUMS
```

有界检查不覆盖全幅闭合或任意物体。手部目标误差作诊断；不会将旧路径包络或0.02rad持握误差当作通用失败。保留数值、限位、Mimic、机械臂跟踪及接触健康检查；不增加训练、参数扫描或几何调优。

- 左手：[固定环境与原生LIBERO复现入口](examples/libero_reference/README.md)、[已验收结果](examples/libero_reference/evidence/RESULTS.md)。5750物理步，连续1.5s持握；相对沉降位置抬升约103.95mm。
- 右手：[B＋functional_v1结果](examples/libero_eval/evidence/functional_v1/RESULTS.md)。6500物理步，连续1.5s无环境支撑对向持握；相对沉降位置抬升约100.49mm。
- [左右统一LIBERO记录接口](examples/libero_eval/README.md)仍保留。历史场景Reset、控制器和动作是抓持结果的一部分，裸MJCF本身不包含白碗、策略或任务环境。

两次白碗实验的原生柜顶放置任务结果均为false；中间抓持能力不等于完整任务成功。历史FAIL、数值附查FAIL和高保真UNKNOWN保留。本轮没有重跑抓取，也不宣称独立手单独执行过白碗任务。

## 来源、许可与发布状态

NERO源描述采用MIT，DH116厂家源采用Apache-2.0；网格来源和逐文件搬运映射见 [来源清单](SOURCE_MANIFEST.json)、[右手源清单](RIGHT_SOURCE_MANIFEST.json)、[四入口资源清单](docs/assets.json)及[许可说明](licenses/SOURCES.md)。厂家法兰的独立许可仍未确认：组合资产不可作无保留的公开授权发布声明。独立手不含NERO或法兰资源。

当前为Issue #6的研究交付候选，需交付审查；PR #4保持Draft，未合并main、未发布Release。原[左手模型说明](docs/MODEL.md)、[静态查看器](examples/view_static.py)与全部历史证据保留。
