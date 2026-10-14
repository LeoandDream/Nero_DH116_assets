# NERO × DH116 — MuJoCo 仿真资产

松灵 NERO 七轴机械臂与 DH116-L000-A1 左手的机器人模型包，提供可移植 URDF、MJCF、可视网格与分块碰撞代理，面向 MuJoCo 场景搭建及 robosuite / VLA 工程接入。

![NERO 与 DH116 参考姿态](examples/preview.png)

## 模型特性

| 特性 | 内容 |
| --- | --- |
| 机械结构 | NERO 7 个关节；DH116 11 个关节、6 个主动通道、5 组线性 Mimic |
| 动力学建模 | 源 URDF 质量、惯性、关节轴与限位；机械臂力矩驱动、手部位置伺服及接触约束 |
| 碰撞表示 | 可视与碰撞网格分离；手部 76 块、机械臂 51 块、连接件 8 块凸代理 |
| 控制契约 | 18D 关节位置状态；7D 机械臂绝对关节位置＋6D 独立归一化手位置 |
| 来源追溯 | 源 URDF、网格映射、冻结场景 SHA256 与交付文件校验清单 |

**已完成来源模型的阶段性仿真验证。** 当前资产提取自已执行的 76 块手部代理场景：在原生 LIBERO 白碗场景中，使用既有位置控制器完成闭合、抬升与持续对向持握；运行 230 个 13D 命令、5750 个物理步，末段连续持握 1.5 秒，无数值警告。结果适用于记录中的模型、控制器和近似条件；不是完整 LIBERO 放置任务成绩或实机标定。详细结果和版本对应见[验证说明](docs/VALIDATION.md)。

## 仿真演示

[76 块代理：白碗闭合与抬升视频][video76] · [68 块代理：同条件对照视频][video68]

两段视频均来自实际物理运行的记录。68 块版本也完成过抬升，不能将该对照描述成“旧版失败、新版成功”。早期失败案例及其条件说明见[验证与演示](docs/VALIDATION.md)。历史媒体和原始证据位于原项目归档，访问需要该仓库权限；本仓库附图为静态参考姿态。

## 快速开始

使用 Python 3.11，安装固定 MuJoCo 3.2.3 的依赖：

```bash
python -m pip install -r requirements.txt
python examples/view_static.py
```

查看碰撞代理、挂载座，或在支持 EGL 的无桌面环境导出图片：

```bash
python examples/view_static.py --collision
python examples/view_static.py --collision --mount
MUJOCO_GL=egl python examples/view_static.py --image preview.png
```

`view_static.py` 只显示参考姿态，不推进物理时间；这描述的是查看工具的用途，不表示来源模型只做过静态验证。

## 接入与控制

| 接口 | 含义 |
| --- | --- |
| Robot state | NERO 7 个＋DH116 11 个关节位置，18D，单位 rad |
| Robot action | 前 7 维为绝对关节目标角 rad；后 6 维为独立绝对手位置目标，范围 [-1, 1] |
| MJCF `data.ctrl` | 前 7 维为电机力矩 Nm；后 6 维为主动手关节目标角 rad |

**不能把 13D robot action 直接写入 `data.ctrl`。** 已验证的执行组合是原生 robosuite JointPositionController 的 `set_qpos` 绝对入口与 MuJoCo position 伺服。下游工程需保留对应适配；本包不附完整控制器启动环境、厨房任务或训练工程。通道顺序见[接口契约](docs/robot-contract.json)。

## 文件与文档

```text
urdf/       源描述与可移植 URDF
mjcf/       组合机器人模型
meshes/     可视网格、碰撞代理与纹理
examples/   静态查看场景与工具
tests/      静态查看回归检查
docs/       模型规格、验证结果与接口契约
licenses/   来源与许可
```

[模型与动力学建模](docs/MODEL.md) · [验证结果与演示](docs/VALIDATION.md) · [来源与许可](licenses/SOURCES.md)

```bash
sha256sum -c SHA256SUMS
python -m unittest discover -s tests -v
```

以上命令检查交付文件完整性及已有静态测试，不重新执行历史动态实验。网格与源场景映射见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json) 和 [URDF_SOURCES.json](URDF_SOURCES.json)。真实驱动上限、安装及相机标定仍未完成；接触代理与真实材料行为并不等同。

[video76]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/moderate-contact-tip-proxy/native-command-comparison/closure-side.mp4
[video68]: https://github.com/LeoandDream/nero_dh116_sim/blob/0b0a316cd4e7b17471cb1827bf2b744bb1a8d80a/experiments/EXP-URDF-FIDELITY-001/reports/benchmark-grasp-diagnostic/smaller-bowl-selection/white-bowl-closure-diagnostic-prepared/closure-side.mp4
