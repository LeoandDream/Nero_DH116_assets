# Issue #5：右手 close149 驱动配置包 B

**唯一一次真实B运行在第245步（0.490s、settle_preshape）触发原路径包络保护并停止，退出码2；没有重试。没有到达接近、close、lift或hold，不能证明抓持改善。**

授权：[6297636306](https://github.com/LeoandDream/Nero_DH116_assets/issues/5#issuecomment-6297636306)。代码/候选基线`43bc90b8932cb2199ad7b19ca00611259d3f7d49`。A直接复用[原close149的4793步](../close149/RESULTS.md)，未重跑A或左手。

## 接入与真实预检

新入口[`close149_drive.py`](../../../right_hand/close149_drive.py)复用原close149、原研究规则、原记录与双相机。未改旧runner。候选仅采用冻结的6个主动通道kp/kv与11个hinge damping/armature，共34个XML属性；这是一组多参数驱动配置包，不是单一kp因果实验。

原字节补丁副本保存在[`raw/drive-model/original-context.xml`](raw/drive-model/original-context.xml)，SHA与冻结候选一致；实际运行的[`model.xml`](raw/drive-model/model.xml)只另外将file引用显式解析到原finger22_local资源上下文，没有复制或改写任何mesh。源路径、实际解析路径和逐资源SHA见[`drive-materialization.json`](raw/drive-materialization.json)。正式资产未改。

- B正式候选→实际LIBERO模型：原完整robot_audit通过，9671项检查、差异0；实际B与历史A完整snapshot仅有预定驱动字段变化。原始B快照未归一化或伪装成A，见[`robot-physical-preflight.json.gz`](raw/robot-physical-preflight.json.gz)、[`old-to-new-compiled-audit.json`](raw/old-to-new-compiled-audit.json)。81手/306腕、质量/完整惯性、轴/限位、线性Mimic、接触/过滤、力限幅、NERO、求解器和相机保持原定义。
- armature派生量单独核查：11个dof_M0满足原armature差；18个dof_invweight0、26个body_invweight0及13个actuator_acc0改变，全部属于机器人，环境项不变。meaninertia由0.0597239224586变为0.0561755353618，见[`drive-derived-preflight.json`](raw/drive-derived-preflight.json)。校验使用反向恢复34属性的A场景**仅编译一次**；先保留原编译姿态，再向独立A model赋值两个原生Reset fixture位姿，完整inventory与历史A相等。没有对A调用Reset、forward、setConst或step，也未写B实时状态。dof_M0的64eps算术界只用于验证浮点加减来源，不改变任何物理门槛。
- 完整Reset的qpos/qvel/ctrl、JPC初始化、criteria及native_scene与A相同；未继承A的warmstart或力缓存。B初始化缓存独立保存在[`drive-native-reset-cache.json`](raw/drive-native-reset-cache.json)，历史A无对应缓存，比较保持UNKNOWN。
- 260条动作原字节SHA与canonical SHA均不变；每宏25步、dt=2ms、RK4、最多6500步、lift首条重复起点及V1全部常数/跨阶段接触历史保留。运行前检查无重复B输出或相关活动进程，原子创建新尝试目录；[实际argv与退出码](raw/execution.json)记录只启动一次。

## 实际结果与A对照

| 项目 | A：原close149 | B：本轮 |
| --- | --- | --- |
| 实际物理步 / 停止阶段 | 4793，lift | 245，settle_preshape |
| 首个阻塞 | LOADED_DEPTH_GROWTH | OUTSIDE_REGISTERED_PATH_ENVELOPE |
| 第245步finger11实际 / 目标(rad) | 0.111029447 / 0.127144136 | 0.147164273 / 0.127144136 |
| 第245步finger11目标误差(rad) | 0.016114689 | 0.020020137 |
| 第245步finger11缓存执行器力矩(N·m) | +0.380317608 | −0.017542656 |
| 前245步臂误差最大(rad) | 0.004128867 | 0.005560534 |
| 前245步主动关节误差最大(rad) | 0.026344681 | 0.021999294 |
| 离桌 / 合格持握 | 未实现 / 0s | 未到达抓取 / 0s |
| 原生柜顶任务 | false | false |

第245步唯一越界为`gripper0_finger11`：冻结全路径上界0.14714413600034068rad，实际0.14716427280610606rad，超出0.00002013680576538rad。相对当时目标的超调是0.0200201368rad；244步及此前18关节均仍在包络内。该保护是既有静态适用姿态域，不是机械关节限位、接触停止或数值发散。B从第1步已可与A不同；同一步外部action/target完全一致，不要求物理前缀相等。

B的245步中无任何合资格机器人接触或手—碗接触。最大Mimic残差0.000777059rad，最大限位超出0.000005216rad；没有非有限值、MuJoCo warning或意外腕部接触，其他原保护均未触发。这里的整体control_health=FAIL包括路径包络停止；不得把未到达的hold误差0解释成hold通过。B末步相对Reset高度−70.589367mm，是白碗自由沉降；未运行至原250步沉降基准，**相对沉降高度为null/UNKNOWN**，不能用245步位置替代该基准。

**迁移/编译/Reset：PASS。数值及其他独立保护：已保存前缀PASS。路径域：FAIL。研究接触规则：未进入手—碗接触。限定抓持：UNKNOWN_EARLY_STOP。原生任务：false。正式高保真、旧2mm与历史FAIL全部保留。**本配置包在当前冻结动作和保护下未达到目标；不能由更小的力推断改善，也不能归因某单独参数。

## 可复核证据与边界

[`raw/`](raw/)保存245行状态、接触、执行器力、完整RK4 body缓存、实际B模型、执行源码/diff、初始化、依赖版本与来源SHA。[`episode/`](episode/)复用原统一导出器，包含245条统一记录；仅显式纠正本次新manifest中原导出器硬编码的A模型身份，绑定实际B XML/profile，旧导出器未改。该纠正及原始身份均保存在manifest中，未重积分或修改轨迹。

两相机各10帧/20fps：[agentview](episode/videos/agentview.mp4)、[手腕相机](episode/videos/robot0_eye_in_hand.mp4)。末帧为245早停，不满一宏；精确时间见[cameras.json](episode/cameras.json)。post-step q/qvel与RK4末内部阶段接触/力缓存不精确对时，不用于强验力矩恒等式。

显式episode积分245步；构造1、显式原生Reset1，初始化刷新及编译器内部自检与episode步数分开记录。结果中的forward计数是不推进时间的API调用，包含原控制链刷新，不能全部称为初始化调用，也不宣称覆盖编译器内部调用。额外A参考只有一次编译、零episode步。

[`tests.log`](tests.log)为8项新B离线检查与8项原close149测试；[`independent-review.json`](independent-review.json)独立核对真实245步及编译变化；[`verification.json`](verification.json)记录旧文件/资源/README保护，SHA清单覆盖本次证据。没有追加烟测episode、调参、减速、阈值或几何修改。

**唯一下一步：先审阅finger11无接触预整形的真实超调及整包驱动迁移响应，再决定是否另行授权新的对照；本轮停止，不自动放宽包络或继续试跑。**
