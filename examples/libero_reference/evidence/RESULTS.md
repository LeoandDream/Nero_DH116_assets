# D1-B 独立动态参考测试验收

2026-10-17，使用正式资产基线 `73483309e16ed812d0b28ef2cdcfca8ecbc2b38e`，在独立克隆中完成真实运行。执行代码提交为 `1dd828dbd41fb8bf9e81c96c79cbb3ee43e47b30`；其后交付提交仅补充本目录证据及文件校验清单。

**结论：固定白碗抓持诊断精确复现。** 230条动作、5750个真实物理步；预检5874项无差异；逐步数值、结构和原始轨迹字节均与 D1-A / 历史基准一致。没有修改正式机器人模型、物理参数、动作、初态、停止条件或比较容差。这是固定 seed 的诊断复现，完整 LIBERO 柜子放置任务未执行。

## 实际运行与独立性

验收克隆位于 `/data_robot/verification/nero_dh116_d1b`，Git对象无 alternates。Python解释器和虚拟环境重新创建；仅复用了经过哈希检查的安装包和独立 LIBERO Git 副本作为显式构建缓存，没有复制已安装环境。

安装和整条验收命令均在以下挂载隔离下执行：系统根只读，独立克隆可写，整个 `/data_robot/projects` 与旧 `/tmp` 被空 tmpfs 覆盖。公开 bootstrap 的默认来源仍是公共 GitHub/PyPI。

```bash
bwrap --ro-bind / / \
  --bind /data_robot/verification/nero_dh116_d1b /data_robot/verification/nero_dh116_d1b \
  --tmpfs /data_robot/projects --tmpfs /tmp --dev-bind /dev /dev --proc /proc \
  --chdir /data_robot/verification/nero_dh116_d1b \
  /bin/bash runs/validate-isolated.sh
```

实际脚本见 [validate-isolated.sh](validate-isolated.sh)。安装、运行、测试、校验和视频流程均 exit0。`bwrap`/`strace`仅用于此次隔离验收，普通运行按[入口说明](../README.md)执行。

- [isolation.json](isolation.json)：运行开始时两个被覆盖目录均为空，PYTHONPATH未设置。
- [independence.json](independence.json)：23020行文件系统调用记录中，0次尝试访问被隐藏的工程路径；185项引擎场景资源全部位于独立克隆中。
- [environment.json](environment.json)：独立解释器、全部导入路径、完整包版本、1019项 stock robosuite文件及1116项 LIBERO文件验证，EGL渲染通过。
- [main-protection.json](main-protection.json)：原main工作树的HEAD、README SHA和未提交状态前后完全一致。

## 结果

| 检查项 | 实际结果 |
| --- | --- |
| 控制／真实物理步 | 230／5750；2ms、RK4、11.5s |
| 正式机器人定义 | `mjcf/nero_dh116.xml`，SHA256 `c446124b7d938aa6e00ba5914047001de66c2a2c162e7dea1644437e5a85cb33` |
| 碰撞配置 | 76手部＋51机械臂＋8法兰分块代理；额外3个挂载座碰撞体全部保留 |
| 预检 | 5874项精确比较；物理字段、命名绑定、完整Reset、控制器缓存无差异；预检0物理步 |
| 动作 | 冻结配置＋11节点计划生成；与基准5750行展开动作差异0 |
| 逐步差异 | `first_difference=null`，所有记录字段最大数值差0 |
| 轨迹字节 | SHA256 `880581b82882313c4abc4c9c838402be6ed04eac18dd5c30ee3780258a27fafa` |
| 相对Reset最终增高 | 33.36294243170235mm，参考 z=0.97m |
| 相对沉降最终抬升 | 103.95223232528839mm，零起算行249，z=0.8994107101064139m |
| Mimic峰值残差 | 0.0010005620208883587rad |
| 最大机械臂目标误差 | 0.07815017042475991rad |
| 手—碗最大代理交叠 | 0.5797042681449684mm |
| 保持阶段 | 750/750步对向接触；完整接触记录中750步手部接触、0步碗与其他物体接触 |
| 数值警告／停止原因 | `{}`／`null` |
| 原生任务／严格抓取声明 | `native_task_result=false`／`strict_grasp_success_claim=false` |

[run-result.json](run-result.json)、[comparison.json](comparison.json)、[metrics.json](metrics.json)、[runtime-preflight.json](runtime-preflight.json)、[source-preflight.json](source-preflight.json)保留实际输出原文。证据中的绝对路径仅记录此次运行位置，运行代码不读取本目录。

环境固定 Python3.11.16、MuJoCo3.2.3、stock robosuite1.4.1、NumPy1.26.4、LIBERO `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`；锁文件和公开来源见 [env](../env/README.md)。实际独立环境中通过41项参考测试、1项原静态测试、3项环境测试及 `pip check`；原始日志随本目录保存。

## 最小运行文件与用途

| 文件／资源 | 用途 |
| --- | --- |
| 正式 `mjcf/nero_dh116.xml`＋实际引用的 `meshes/` | 唯一机器人定义；arm/hand/mount接口片段均从此派生 |
| 正式 `urdf/dh116_left.urdf` | 记录器的原始Mimic及手目标误差口径；不作为第二份机器人模型 |
| `run.py`、`model.py` | 原生LIBERO装配、绝对位置适配、两次seed/reset、执行与输出 |
| `actions.py`、`diagnostic.py` | 原动作生成与原诊断停止/记录逻辑；不重新规划、不注入物体状态 |
| `checks.py`、`compare.py` | 执行前精确核对、逐步差异与两个高度基准 |
| `fixtures/` | 冻结动作输入、比较用物理/Reset/控制器快照、明确的压缩验收基准和来源哈希 |
| `env/`＋其公开固定依赖 | 独立Python与依赖重建；原生LIBERO白碗BDDL及场景资源随该固定LIBERO获取 |
| `render_video.py`（可选） | 从实际记录状态生成视频；只调用mj_forward，新增物理步0 |

`docs/robot-contract.json`、SOURCE/URDF Manifest及源网格也用于完整性预检。无需历史完整场景XML、历史MJB、旧虚拟环境、策略权重、训练数据或实验脚本目录。

## 实际轨迹、接触与视频

- [bounded_grasp.jsonl.gz](bounded_grasp.jsonl.gz)：此次真实输出压缩副本；解压SHA见上表。由于真实输出逐字节一致，其压缩内容也与随附基准一致。
- [contact-details.jsonl.gz](contact-details.jsonl.gz)：5750步完整接触geom/body、距离、接触系力及执行器力。
- [closure-side.mp4](closure-side.mp4)：720×540，230帧，20fps，11.5秒；实际轨迹状态重渲染。来源与ffprobe结果见 [closure-side.provenance.json](closure-side.provenance.json)。
- [artifacts.json](artifacts.json)：源运行相对路径、原文SHA、压缩SHA及体积。完整引擎XML和原始strace/安装日志留在独立验收输出中，只记录哈希，不向正式资产增加另一套机器人定义或开发流水账。

仅覆盖此固定场景、seed和动作；不代表多seed Benchmark、完整任务成功、其他接触工况或实机标定。跨机器发生精确差异时应保留报告并定位首个差异，不据此调整参数或放宽容差。
