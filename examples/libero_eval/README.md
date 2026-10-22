# NERO + DH116-L/R LIBERO 最小评测入口

统一任务身份、state/action、Reset 来源、阶段、原生任务结果和图像索引；两只手保留各自已经验证的机械参数、模型和动作。运行环境复用 [固定依赖](../libero_reference/env/README.md)：Python 3.11.16、MuJoCo 3.2.3、stock robosuite 1.4.1、LIBERO `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`。

本轮真实结果见 [Issue #5 验收记录](evidence/RESULTS.md)：左手复用5750步基线；右手唯一尝试在4775步由研究接触规则停止，未进入lift/hold。两手原生柜顶放置判定均为false。

默认只读取真实记录，纯 Python 标准库，不导入 MuJoCo、不编译模型、不 Reset 或积分：

```bash
python examples/libero_eval/run.py --hand L --mode baseline --output runs/eval_left
python examples/libero_eval/run.py --hand R --mode research \
  --recorded-source PATH_TO_NEW_RESEARCH_RUN --output runs/eval_right
python examples/libero_eval/run.py aggregate runs/eval_left runs/eval_right \
  --output runs/eval_summary.json
```

L 默认导出本仓库已经验收的 5750 步／230 条动作；R `baseline` 默认导出原 4769 步历史早停记录，`research` 必须显式提供新研究运行目录，不能把旧 FAIL 改写成新运行。输出必须不存在；不会覆盖历史记录。重复导出同一次运行只用于格式转换，不能作为独立 episode 增加成功率样本。

只有显式 `--execute` 才调用既有后端。L 使用原 `libero_reference/run.py`；R 使用独立 `right_hand/libero_research.py` 的一次性冻结研究入口。真实记录先保存到输出旁的 `OUTPUT-raw`，再导出到 `OUTPUT`。本工具不自动重试，不循环 episode，不调参数；实际运行还必须符合当前任务授权的一次实验额度。

```bash
# 会进行真实积分；不要在只读分析或未授权实验时运行。
python examples/libero_eval/run.py --hand R --mode research --execute --output runs/eval_right_once
```

每个导出包含：

- `manifest.json`：模型、Reset、动作记录及任务来源 SHA；Python/依赖身份；研究规则 SHA。
- `steps.jsonl.gz`：每真实物理步的 18D 关节状态、13D 动作、13D ctrl、阶段、接触、数值警告和原始完整状态。R 保留保存的 RK4 缓存时序和 body poses；L 原日志没有触点位置/frame/body poses，不会补造。
- `result.json`：实际／计划步数、完整执行或早停、控制与限定抓持原始结果、原生任务结果以及两个独立高度参照。`simulator_health` 分开列逐步有限值、时间和数值警告检查及旧保护阈值，未保存的证据为 `UNKNOWN`；不能由普通完成推定全部保护通过。`new_physics_steps=0` 指导出本身；不抹去源运行实际积分步数。
- `cameras.json`：实际存储图像、视频及时间索引。未记录的逐步图像／原生成功信号为 `null`。L 优先复制本次源目录的最后一帧 agentview/wrist 图像；只有默认已验证轨迹及 SHA 绑定通过时才引用已有历史末帧。源缺失的相机不会借用别次运行。侧视视频不会冒充两路原生相机轨迹。R 复制源 `frames/`、`videos/`，保持相机原名。

机器人 state 是 7 个臂关节＋11 个手关节位置；action 是 7D 绝对臂角度＋6D 归一化绝对手位置；ctrl 是 7D 力矩＋6D 主动手关节角度。源数据仍保留各手自己的关节顺序与动作数值。

右手每步同时保留 `legacy_failures` 与 `research` 双通道，不覆盖原正式规则的失败记录。`--execute` 在成功导出后仍返回后端原始退出码：早停／错误不会被格式转换掩盖为退出0。原生任务 `false` 本身不等于程序执行错误。

初始化使用 seed `20261007` 的原生 LIBERO Reset，保存完整实际快照；没有加载 Panda 的官方 init-state 文件，也没有实现不同机器人间的官方 init-state 转换。Reset 后的动作探针和第二次 Reset 仍由原后端负责，导出工具不执行这些操作。

最终任务指标来自原生 `env.check_success()`：把白碗放到柜子顶层。现有抓取路径只覆盖 settle/approach/descend/close/lift/hold，没有柜子放置阶段；白碗离桌、抬升、持握均为中间能力，不会替代完整任务成功。`strict_grasp_success_claim=false` 是不作额外严格抓取声明，不等于独立抓取谓词失败。

汇总按“手型＋源轨迹原始路径＋SHA256”拒绝重复；即使导出目录或 episode 名称不同，同一源记录也不能重复计数。确定性独立运行可能产生相同轨迹 SHA，因此不同原始运行目录可以具有相同 SHA；路径不同或 SHA 相同本身均不能证明执行尝试独立，统计附带此限制。缺少唯一源轨迹路径及 SHA 时拒绝生成统计。汇总同时报告完整 episode 的 `n_success/n_episodes`、`success_rate`，以及包括早停尝试的 `attempted_success_rate`；truncated/incomplete 单独计数。不得只报告剔除失败早停后的条件成功率。当前结果属于自定义 NERO + DH116 的固定场景科研实验，不是官方 leaderboard、多 seed 或独立重复试验统计。

离线检查：

```bash
python -B -m unittest discover -s examples/libero_eval -p 'test_*.py' -v
```

正式模型、旧运行器、原 2mm 标准、历史 FAIL 和高保真 UNKNOWN 均不由此入口变更。后续固定初始状态的重复试验需单独冻结 episode 清单并获得实际运行授权。
