# Issue #6 — 四套研究仿真资产交付验收

**结论：四套入口均达到本轮研究运行交付要求，等待交付审查；不是高保真认证或Release。** 左右接口已对齐；原左手V1与右手成功B模型的机械/驱动/接触字段没有修改。本轮没有新白碗rollout、训练、调参或代理生成。

| 交付入口 | 新增实际测试 | 本轮状态 | 抓持证据的适用范围 |
| --- | --- | --- | --- |
| `mjcf/dh116_left.xml` | 独立加载、6通道2400物理步 | PASS | 独立固定掌部控制；没有独立手白碗成绩 |
| `mjcf/dh116_right.xml` | 同上，右手机械＋B驱动 | PASS | 同上；不继承旧17Mesh右手的版本声明 |
| `mjcf/nero_dh116.xml` | stock JPC＋手部400步 | PASS；V1原字节不变 | 复用原5750步白碗、1.5s持握 |
| `mjcf/nero_dh116_right_research.xml` | stock JPC＋手部400步 | PASS；与成功B编译等价 | 复用7998718的6500步、约100.494mm沉降基准抬升、1.5s持握 |

机器可读总结果：[acceptance.json](acceptance.json)。最终四模型合计新增5600个2ms控制步，另保留初版独立手各1步的失败记录。不把初始化编译器自检算作记录轨迹；不声称原生库内部绝对零积分。

## 模型和接口对应

- 正式左手V1 MJCF SHA256：`c446124b7d938aa6e00ba5914047001de66c2a2c162e7dea1644437e5a85cb33`，与main基线9b5ee0d一致。
- 右手唯一源：7998718中 `examples/libero_eval/evidence/functional_v1/raw/drive-model/original-context.xml`，SHA256 `e24d17904b9b1d4a55ae031a08bb1457f0f46c4170f987767d0acd161eb3e84b`。原资源解析语境是 `examples/right_hand/candidates/finger22_local/`，并非归档XML所在目录。
- 新右组合仅改变model标签和资源路径。351组公开编译数组精确相等，覆盖网格顶点/面/凸图、质量惯性、关节、执行器、约束、接触、keyframe及派生物理量；460份编译Mesh保留。命名物理快照与当年运行保存的formal_snapshot精确相等。仅排除名称查找和文件路径地址元数据，未使用容差。
- 两独立手分别保留对应组合的18个手部body、11关节、6驱动、5组线性Mimic、3个site、2个sensor，手部本地机械字段与编译网格精确相等。固定掌部于原点，移除NERO/法兰；不引入浮动关节，不改质量、轴、行程、惯性或伺服参数。
- 左右手外部顺序/单位相同：独立手11D state / 6D归一化绝对action，组合18D / 13D（7臂绝对角度rad＋6手）。独立手不伪造NERO状态。原 `docs/robot-contract*.json` 保留，四入口见 `docs/assets.json`。
- 506份资源逐文件SHA和原始路径记录在索引中；306腕和既有手部代理均原字节复用，未生成新几何。运行仅需稳定MJCF与meshes，不读历史候选或临时LIBERO目录。

来源核验：[verified-source-equivalence.json](verified-source-equivalence.json)。显式比较命令（不属于运行链）：

```bash
python tools/check_asset_sources.py --output runs/source-equivalence.json
python -m unittest discover -s tests -p test_four_assets.py -v
```

## 固定掌部的实际适配与保留FAIL

初版直接固定掌部后，左右手首步均出现约11.46mm掌部—近端指节交叠并停止。原因不是新增几何误差：[MuJoCo 3.2.3 filterBodyPair](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_collision_driver.c#L135-L151)对world焊接体不执行父子过滤，导致原组合自动过滤的五组相邻装配、共50对geom重新参与碰撞。

最终只在独立手显式保留这五组原父子关系：base_link ↔ finger11/21/31/41/51_link。不是新增关闭非相邻自碰撞，不改变组合模型。逐对资格核验：左2311→2311、右2613→2613；没有删除任何原合资格内部geom对，contype/conaffinity及外物接触资格不变。

[修复前资格差异](fixed-base-contact-policy-before.json)、[初版模型及语境](initial-fixed-palm/CONTEXT.json)、[原始首步FAIL及组合PASS](isolated/control/results.json)全部保留。早期source比较还曾报告tex_pathadr地址差异；它是路径字符串地址元数据，实际物理数组相等，原输出也保留。没有改写历史FAIL或放宽物理阈值。

## 隔离控制检查

验证环境：CPython3.11.16、MuJoCo3.2.3、NumPy1.26.4、stock robosuite1.4.1。仅复用只读已安装SDK；未声称本轮重新pip安装。将四模型、资源、索引和两个控制脚本复制到干净包，以Linux mount namespace隐藏整个 `/data_robot/projects` 及旧验证工程（仅重新挂入Python解释器和site-packages），因此历史目录、LIBERO和旧运行目录不可访问；PYTHONPATH清空。无图形/相机依赖。

首次物理步前写入协议。独立手逐通道100步增加目标、100步保持、100步返回、100步保持，共2400步；幅度为actuator全行程的10%，其他通道保持参考目标。组合为400步同样手部小幅目标，同时J6增加0.01rad再返回。有界测试每物理步更新平滑目标；示例API默认每命令保持25步。此检查与20Hz原白碗动作日程分别记录，不是全幅开合测试，不扫描幅度或增益。

| 指标 | 独立L | 独立R | 组合L | 组合R |
| --- | --- | --- | --- | --- |
| 六通道均观测到正确方向响应 | 是 | 是 | 是 | 是 |
| Mimic最大残差/rad | 0.000752 | 0.000734 | 0.000613 | 0.000696 |
| 最大软限位超出/rad | 0.000618 | 0.007820 | 0 | 0 |
| 机械臂最大目标误差/rad | — | — | 0.015240 | 0.009053 |
| MuJoCo数值警告 | 0 | 0 | 0 | 0 |

使用原Mimic0.02rad、限位0.01rad、臂跟踪0.1rad及2mm接触健康界限；右独立手软限位偏离真实存在但未超原界限，未调参掩盖。手目标误差仍完整记录（峰值约0.034–0.047rad），不将旧0.02rad持握误差升级为通用硬失败。此无物体小幅控制中没有实际接触；组合基础接触能力由物理等价绑定的白碗原始证据支持。

[独立手最终结果与协议](isolated-final/control/results.json)、[组合结果及协议](isolated/control/results.json)、[五项契约测试](contracts-tests.log)。逐步trace均含action、post-step状态/速度、ctrl、Mimic及实际接触/力；contact为mj_step最后RK4内部阶段缓存，未用事后forward替代。测试脚本拒绝覆盖输出，无自动调参/重试。

复核控制（创建新的输出，不重用证据目录）：

```bash
python examples/check_assets.py --output runs/four_assets_001
# 仅测独立手：加 --assets dh116_left dh116_right
```

## 工程取舍、范围与交付阻塞

参考[robosuite1.5.2实际Inspire MJCF](https://github.com/ARISE-Initiative/robosuite/blob/v1.5.2/robosuite/models/assets/grippers/inspire_right_hand.xml)及其[动作适配](https://github.com/ARISE-Initiative/robosuite/blob/v1.5.2/robosuite/models/grippers/inspire_hands.py)中资源与接口分离的做法；[DexMimicGen官方工程](https://github.com/NVlabs/dexmimicgen)提供任务环境和独立回放入口。本轮只借鉴分层，不复制其他手的机械参数，不升级1.4.1、不迁移环境、不追求全域数字孪生。

- 右手成功只对应B＋functional_v1与冻结81手/306腕、close149动作及原完整LIBERO Reset。左手同样只对原固定场景/控制/动作有证据。交付裸MJCF不会自动重建任务或策略。两原实验native柜顶任务结果false，抓持不等于完整任务成功。
- 原全幅FAIL、纳米数值附查FAIL、高保真UNKNOWN保持；独立手任意朝向/全行程/任意物体尚未验证。它们不作为当前有界研究交付的新增门槛。
- **技术功能上未发现阻止本轮四入口研究使用的问题。** 独立手不含法兰；组合模型仍有厂家法兰独立许可缺项，阻止无保留公开授权发布。交付审查仍待完成，不合并、不Release。
- main、用户未提交README、左手V1、原正式右模型、共享旧网格和历史证据逐文件保护核对见 `protection.json`。新README仅写在隔离分支。
