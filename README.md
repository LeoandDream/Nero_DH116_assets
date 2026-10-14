# NERO × DH116 — MuJoCo 仿真资产

**松灵 NERO 七轴机械臂 + 左侧 DH116 多指灵巧手**的可移植机器人模型包。面向 MuJoCo / robosuite 场景搭建，提供来源可追溯的 URDF、MJCF、可视与碰撞网格，以及明确的状态和动作接口约定。

![NERO 与 DH116 参考姿态](examples/preview.png)

## 模型特性

| 模块 | 内容 |
| --- | --- |
| 机器人组合 | NERO 7 关节 + DH116-L000-A1 左手 11 关节（6 个主动通道、5 个联动关节） |
| 动力学描述 | 刚体质量与惯性、关节限位、驱动器、Mimic 约束及 MuJoCo 接触模型 |
| 碰撞建模 | 可视网格与接触网格分离；分块凸代理：灵巧手 76 块、机械臂 51 块、连接件 8 块 |
| 数据接口 | 18D robot state、13D robot action 的顺序和语义约定 |
| 可移植性 | 模型依赖、原始 URDF、网格资源、来源清单和文件校验和一并提供 |

本项目交付的是**机器人仿真模型资产**，不是完整的抓取任务、策略训练框架或实机控制 SDK。模型定义包含动力学与接触参数；这些参数的存在不代表已完成真实硬件标定或针对全部操作任务的动力学验证。

## 快速开始

建议使用 Python 3.11；依赖中固定 MuJoCo 3.2.3。

```bash
git clone https://github.com/LeoandDream/Nero_DH116_assets.git
cd Nero_DH116_assets
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt

# 显示参考姿态
python examples/view_static.py

# 可视化碰撞代理
python examples/view_static.py --collision

# 同时显示挂载座
python examples/view_static.py --collision --mount
```

无图形桌面的 Linux 环境可以尝试导出静态预览图：

```bash
MUJOCO_GL=egl python examples/view_static.py --image preview.png
```

检查交付文件完整性、运行仓库提供的静态回归测试：

```bash
sha256sum -c SHA256SUMS
python -m unittest discover -s tests -v
```

以上示例用于**模型加载、结构检查与静态查看**，不会调用物理时间步进或执行机器人动作。

## 接入与控制

| 接口 | 约定 |
| --- | --- |
| State | NERO 7 个关节位置 + DH116 11 个关节位置，共 18D |
| Robot action | NERO 7 个绝对关节目标角（rad）+ DH116 6 个独立归一化绝对位置目标，共 13D |
| 原生 MuJoCo ctrl | 前 7 维为力矩控制输入，后 6 维为手指位置伺服目标；**不能直接写入 13D robot action** |

下游仿真程序需要提供关节位置控制器与动作转换层；本仓库不附带完整的 13D 动作运行环境。状态顺序和通道定义见 [robot-contract.json](docs/robot-contract.json)。

## 项目内容

```text
urdf/       NERO 与左 DH116 的源描述及可移植 URDF
mjcf/       NERO + DH116 MuJoCo 机器人模型
meshes/     可视网格、凸碰撞代理和相关纹理
examples/   静态场景与预览工具
tests/      资产静态回归检查
docs/       模型规格、动力学建模与验证边界
licenses/   来源与许可说明
```

### 文档

- [模型规格与动力学建模](docs/MODEL.md)：机械结构、驱动、碰撞代理及物理参数的含义。
- [验证状态与适用范围](docs/VALIDATION.md)：哪些内容已具备交付检查入口，哪些动力学能力仍需独立验证。
- [状态与动作契约](docs/robot-contract.json)：下游接口约定。
- [来源与许可证](licenses/SOURCES.md)：第三方模型与资源归属。

模型网格来源与 SHA 见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)、[URDF_SOURCES.json](URDF_SOURCES.json) 和 [SHA256SUMS](SHA256SUMS)。

## 使用边界

当前组合采用可追溯的连接法兰安装假设，并保留源 URDF 可获得的机械参数。接触几何采用数值仿真的凸近似；摩擦、材料、传动、控制增益、真实额定力矩与速度及实机安装均不能仅由该资产包推定为已校准。
