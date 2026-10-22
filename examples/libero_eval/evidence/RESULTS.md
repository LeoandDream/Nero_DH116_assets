# Issue #5：左右手 LIBERO 最小实验接口与右手单次研究尝试

**实际结论：接口最小版本已交付；右手在第4775步按冻结研究规则停止，尚未完成抓取；两手均没有完成原生柜顶放置任务。** 本轮没有再次运行左手、重试右手、修改几何或调整参数。

## 任务与证据范围

依据 [Issue #5](https://github.com/LeoandDream/Nero_DH116_assets/issues/5)，基线 `85a265ac96f0be141f82d10667b9ea8e8ce02e99`。统一入口 [`../run.py`](../run.py) 分别选择现有左手后端和独立右手研究后端；默认仅转换已保存的真实运行记录，只有显式 `--execute` 才执行一次所选后端。安装、命令和字段见 [`../README.md`](../README.md)。

| 项目 | 左手 L | 右手 R |
| --- | --- | --- |
| 证据 | 复用原已验收白碗记录，本轮零新积分 | 本轮唯一真实研究尝试 |
| 模型 | 正式左手76块手部、51块臂、8块法兰 | 冻结81块手部、306块腕部，其他机器人定义不变 |
| 实际 / 计划物理步 | 5750 / 5750 | 4775 / 6750 |
| 命令 | 230条完成 | 原270条日程；190条返回，下一条在第25个子步早停 |
| 阶段 | 完成 close / lift / hold | close执行775/1000步；lift、hold均未执行 |
| 最终相对Reset高度 | +33.362942mm | −69.962537mm |
| 最终相对本次沉降位置高度 | +103.952232mm | +0.626753mm |
| 持握 | 原记录hold 1.5s，逐步拇指/对侧接触 | 有环境支撑，有效无支撑持握0s；抓持结果UNKNOWN |
| 数值健康证据 | 已记录数组有限、时间和warning通过；未记录qacc有限标志为UNKNOWN | 保存前缀有限、无warning，独立控制保护均未触发 |
| 原生 `env.check_success()` | **false** | **false** |
| 相机 / 视频 | 已绑定轨迹SHA的两路历史末帧＋230帧侧视回放 | 原生agentview、robot0_eye_in_hand共191组 / 382张JPEG，两路191帧MP4 |

两手采用同一 LIBERO90 `KITCHEN_SCENE9_put_the_white_bowl_on_top_of_the_cabinet`，BDDL SHA256 `a595358c840e6dca31d2d914726bb49d58944252ae868796bd603345a444b627`，原生seed `20261007` Reset。各手独立轨迹、机械和控制参数不强行相同。state为18D关节位置；action为7D绝对臂角＋6D归一化绝对手位置；ctrl为7D力矩＋6D主动手关节角。**该13D接口不是官方Panda 7D EEF-delta策略接口。**

## 右手真实运行与首个阻塞

右手使用 [`../../right_hand/libero_research.py`](../../right_hand/libero_research.py)，模型 `finger22_local/model.xml` SHA256 `7f9d84061c3e7d176763c688d2389f8b3d3e38386b8e990a0fa047ff2bc3329c`，冻结规则 `DH116_RESEARCH_CONTACT_V1` SHA256 `48632dfa4bbe0630ef7d61b9e706240d7769ad6361d588e1d02025bde4bc6454`。旧2mm判据继续单独记录，原规则提案文件和所有历史FAIL未改写。

- 原始实际运行：`/data_robot/verification/nero_dh116_right/runs/issue5/right_research_once`；启动argv、时间、输出和退出2见 [`right_raw/execution.json`](right_raw/execution.json)。Python 3.11.16、MuJoCo 3.2.3、stock robosuite 1.4.1、LIBERO `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`，复用仓库已锁定环境。
- 在只读源码/依赖的隔离挂载中运行；原工程路径与旧私人runs不可见。仅新输出目录可写。开跑前核对514份冻结源码/资源，以及实际Reset、控制器、物理编译快照和270条动作。4份旧生成XML只保留溯源SHA，不作运行依赖。
- 前**4769步物理状态和完整RK4接触缓存与上次真实记录完全一致**。所有4775步在线研究判断与独立纯离线回放完全一致，见 [`right-independent-review.json`](right-independent-review.json)。
- 旧判据仍在4769步报 `ROBOT_CONTACT_OVERLAP`。研究入口继续至4775步，`gripper0_finger23_link ↔ white_bowl_1_main` **body聚合**触发 `LOADED_DEPTH_GROWTH`：4769–4775共7个采样，观察跨度12ms；深度2.029398→2.614642mm，净增0.585244mm；窗口平均法向标量载荷4.667805N，高于冻结参考2.331547N。
- 窗口梯形采样载荷暴露0.0560136613N·s；这是缓存采样量，**不是真实连续积分冲量**。停止时body总载荷5.152556N，单g23的1.958328N不足以单独达到带载条件。
- 保存前缀最大臂误差0.088806594rad（限0.1）；Mimic残差0.000278909rad（限0.02）；限位超出0；warning为0。无意外腕部/路径包络失败。源 `control_health=FAIL` 包含研究接触停止，不表示关节或数值发散。

每步保留post-step状态、控制、力、接触位置/法向及RK4最终子阶段body poses；二者时序不能混同。观察/原生成功检查不增加forward，且验证所用状态与求解缓存未变化。offscreen renderer只增加构造期1次forward，之后走完整原生Reset；精确旧前缀比较证明未改变记录的物理轨迹。显式轨迹计数4775步，不宣称计数覆盖编译器内部原生自检。

## 可复核交付与验收

- [`left_001/`](left_001/) 和 [`right_001/`](right_001/)：统一schema，逐步state/action/ctrl、阶段、原始判据、研究判据、Reset来源与SHA、相机索引、结果。未记录的字段保持null，不伪造左手RK4缓存或逐步图像。
- [`right_raw/`](right_raw/)：原始state/contacts压缩文件、全部冻结快照、实际执行源码及5处最小接入diff。大型编译快照仅无损gzip；[`artifact-source.json`](right_raw/artifact-source.json)绑定原始和存储SHA。图像/视频在right_001中原字节保留，不重复存储。
- 视频：[右手agentview](right_001/videos/agentview.mp4)、[右手手腕](right_001/videos/robot0_eye_in_hand.mp4)、[左手历史侧视](../../libero_reference/evidence/closure-side.mp4)。前两段为本轮实际观察帧，后者是已有状态回放，不冒充本轮新积分。
- 纯离线针对性测试、实际数据一致性检查与媒体校验见 [`verification.json`](verification.json)、[`tests.log`](tests.log)、[`right-integration-tests.log`](right-integration-tests.log)。导出和测试不启动引擎。
- [`aggregate.json`](aggregate.json)：左手完成1次、右手早停1次，原生成功均0；完成分母0/1、含早停尝试分母0/2。**这不是独立重复试验成功率，也不是官方LIBERO leaderboard。**

**M1：最小统一入口可用，历史观测缺口明确；M2：唯一右手尝试已执行，完整抓持未达成；M3：仍未完成。** 尚未实现官方Panda init-state到自定义机器人的转换，没有新多episode试验，现有两条路径都没有柜顶放置阶段。不能用白碗中间能力代替最终任务成功。

**唯一优先下一步：**依据本次已保存的close带载增长证据，审查研究停止规则与探索性抓取目标的适配，决定下一次单变量方案后再授权运行；本轮不自动放宽规则、调参、重试或削减几何。正式资产、高保真UNKNOWN/数值FAIL、main和用户README均保留；PR #4保持Draft。
