# NERO + 左DH116 仿真资产

以源URDF为机械依据的机器人资产交接包，包含可移植URDF、MuJoCo模型、visual／collision mesh和最小静态查看入口。

```text
urdf/        独立机械臂、左手、机械臂带临时厂家法兰；source保留原描述
mjcf/        已静态核对的组合机器人模型
meshes/      nero、dh116、mount、textures；保留源mesh和衍生代理来源
examples/    静态查看，默认不显示碰撞代理
docs/        接口、物理近似与缺项
licenses/    随源许可及连接件来源缺项
```

## 查看

使用Python 3.11创建自己的环境，安装`requirements.txt`后：

```bash
python examples/view_static.py
python examples/view_static.py --collision
```

无桌面服务器可导出静态图片：

```bash
MUJOCO_GL=egl python examples/view_static.py --image preview.png
```

查看程序不调用mj_step、不执行机器人动作。原生MuJoCo ctrl前7维是电机力矩，后6维是手主动关节角；不能直接写入13D robot action。正式接口见[控制与状态契约](docs/robot-contract.json)。

## 模型范围

- NERO 7个机械臂关节、左DH116-L000-A1的11个手关节，6个主动通道和5个源线性mimic。
- 原质量／惯性／轴／限位和机械默认值保持；当前碰撞为贴合源mesh的分块代理，visual与collision分开。
- 暂用厂商法兰安装假设；内部过滤仅包含源标记相邻的base_link/link1。真实硬件力矩／速度、安装和相机标定尚不完整。
- MuJoCo position数值参数来自已有仿真验证，**未做实机校准**。静态等价不代表全工作空间或真实抓持性能保证。

这个包包含机器人模型和静态示例；训练、策略、抓持调参、厨房任务及末轮实验记录留在开发历史。[详细限制](docs/MODEL.md)与[来源许可](licenses/SOURCES.md)应随模型阅读。

## 来源与完整性

`SOURCE_MANIFEST.json`记录MJCF依赖来源与SHA，`URDF_SOURCES.json`记录源URI到本地mesh的对应；`urdf/source/`保持源描述原字节。`SHA256SUMS`覆盖交付文件。

```bash
sha256sum -c SHA256SUMS
```

本包尚不提供维护好的JointPositionController环境启动入口；纯资产MJCF不是机器人13D动作执行接口。
