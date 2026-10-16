# 独立 LIBERO 白碗参考测试

运行只需要本仓库与公开固定依赖，不需要实验工程、历史归档、旧虚拟环境或私有仓库权限。
机器人定义始终读取 [`../../mjcf/nero_dh116.xml`](../../mjcf/nero_dh116.xml) 和对应 Mesh；没有另一份机器人模板。

## 安装与运行

Linux x86_64；系统依赖和 EGL 配置见 [env/README.md](env/README.md)。本 Draft PR 可按下面方式检出：

```bash
git clone --branch feature/d1b-independent-reference https://github.com/LeoandDream/Nero_DH116_assets.git
cd Nero_DH116_assets
bash examples/libero_reference/env/bootstrap.sh
source examples/libero_reference/env/activate.sh
python examples/libero_reference/run.py --output runs/white_bowl
```

输出目录必须是本 checkout 内尚不存在的新目录，工具拒绝覆盖。bootstrap 固定独立 Python3.11.16、MuJoCo3.2.3、stock robosuite1.4.1、NumPy1.26.4 和 LIBERO `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`；版本、发行包 SHA256 和完整 LIBERO 文件清单随代码交付。
生成的环境和依赖放在 `.reference-env`、`.reference-deps`，不会装入当前 Python 环境。

## 输出与验收

- `bounded_grasp.jsonl`：5750个真实物理步的状态、动作、内部 ctrl、接触、Mimic、高度和数值警告。
- `contact-details.jsonl`：每步接触 geom/body、距离、接触系力与 actuator force。
- `source-preflight.json`、`runtime-preflight.json`：文件、物理字段、绑定关系、场景 Reset 和控制器缓存的逐项检查。
- `metrics.json`、`comparison.json`：两种高度口径及相对固定验收轨迹的首个精确差异和最大差异。
- `run-result.json`：执行状态、真实步数、控制命令数及原生/严格成功信号。

默认显式验收基准是 `fixtures/golden/bounded_grasp.jsonl.gz`，其解压 SHA256 为 `880581b82882313c4abc4c9c838402be6ed04eac18dd5c30ee3780258a27fafa`。
它只用于命令核对和验收比较，不提供控制输出。动作由冻结的初始配置与11节点计划生成。
其他验收轨迹可用 `--reference FILE.jsonl[.gz]` 显式传入；不自动搜索外部目录。

返回0：完整执行且逐步轨迹与验收基准精确一致。返回2：完整执行或诊断停止后生成了差异报告。返回1：配置、物理预检或执行错误。
不设置浮点容差，也不把结果接近历史数值计为 PASS。原始停止规则保留。

相对 Reset 的最终增高为约33.36mm；相对沉降位置（零起算行249）的最终抬升约103.95mm。
两者参照位置不同。native task / strict grasp 信号保持原定义，参考结果均为 false；这里没有完整柜子放置任务或多 seed Benchmark 成绩。

## 机器人和 Reset 接入

`model.py` 从正式 MJCF 派生 robosuite 所需的 arm、hand、mount 接口片段，只处理名称空间和安装坐标；质量、惯性、关节、代理、接触和驱动均来自正式定义。
挂载座底部接口偏移由正式基座世界高度推导，运行前再次精确核对其世界位姿。
场景使用公开固定 LIBERO 原生厨房任务，任务身份和成功判定继承原类，只显式带入正式 MJCF 的模型级设置。
7D绝对角度通过原生 `JointPositionController.set_goal(..., set_qpos=...)` 计算力矩，6D归一化手目标由原生 gripper 接口映射到角度。

原生构造 Reset、seed/reset、绝对目标探针、第二次 seed/reset 顺序保留。
整个流程直接使用正式 MJCF 的最终参数；不重放旧启动增益、不补写控制器缓存、不注入物体状态。
`fixtures/compiled-model.json` 只有已验证基准的命名数值字段，用于比较；它不能生成或替换机器人模型。

## 可选视频

安装系统 `ffmpeg` 后：

```bash
python examples/libero_reference/render_video.py --run runs/white_bowl
```

生成 `closure-side.mp4` 与来源记录：11.5秒、20fps、720×540。
视频从本次记录状态通过 `mj_forward` 重渲染，禁止物理步进，不构成第二次物理重放。

## 检查与证据

```bash
python -m unittest discover -s examples/libero_reference -p 'test_*.py' -v
python -m unittest discover -s tests -v
python examples/libero_reference/env/test_provenance.py
python -m pip check
```

实际验收见 [evidence/RESULTS.md](evidence/RESULTS.md)。完整环境的第三方系统驱动仍需满足 EGL 条件；固定单 seed 的轨迹一致不代表实机标定、其他物体或全工作空间保证。
