# NERO＋DH116右手候选：实际验收记录

2026-10-17；实现提交`160c8f4076ae39034c28d07c40be9c62c1aa69ac`，基线`main@9b5ee0d38442dcf12bc9f41ec5a0111fb0bd6a7c`。**静态机械检查PASS；真实动力学完整执行，但全幅开合筛查FAIL；基础掌部外物接触筛查PASS；抓持UNKNOWN。** 本结果不属于左手白碗复现，也不是LIBERO任务成功。

## 模型与隔离运行

- 正式入口：`mjcf/nero_dh116_right.xml`，SHA256 `739d3d29bbfd4a6468fb8b75f4de09b719c0f96abb39508959c1dfc33a84b9cc`。18个机器人DOF、13个actuator、5组线性Mimic；右手17个原始link Mesh凸包，NERO51＋法兰8＋挂载座3保持原样。
- 固定右源`14275cfe32cce1fe13a16df712e8a0e610a0227e`；采用右URDF机械/线性Mimic和明确列出的源MJCF仿真参数，不复用左手76块代理。[逐字段源对照](SOURCE_COMPARISON.md)、[源JSON](source-comparison.json)、[装配测量](mount-geometry.json)。相同字段不能证明与二次Mimic/不同碰撞启用方式的上游MJCF整体动态等价。
- 独立克隆`/data_robot/verification/nero_dh116_right`重新解包Python和安装虚拟环境；只显式复用不可变下载归档缓存，没有复制已安装解释器/venv。[依赖重建记录](dependency-rebuild.json)。
- 在bubblewrap内将`/data_robot/projects`和`/tmp`挂为新空目录；源码缓存不存在，Git无alternates。[隔离记录](isolation.json)。Python3.11.16、MuJoCo3.2.3、stock robosuite1.4.1、NumPy1.26.4；stock1019文件、LIBERO1116文件核验通过。[完整版本与import路径](environment.json)。该测试未调用LIBERO环境。
- 使用stock `JointPositionController`保持NERO、正式XML的6个position servo控制右手。协议/动作在首跑前冻结，没有执行中写入qpos、修改机器人参数或增加过滤。[协议及阈值](frozen-protocol.json)、[动作压缩文件](frozen-commands.json.gz)。

## 实际动力学结果

| 项目 | 单通道＋联合开合 | 独立外物探针 |
|---|---:|---:|
| 真实mj_step / 逐步状态记录 | 7000 / 7000 | 2000 / 2000 |
| 仿真时间 | 14s | 4s |
| 非有限状态 / MuJoCo warning步数 | 0 / 0 | 0 / 0 |
| 机械臂最大保持误差/rad（界限0.1） | 0.02746571 | 0.01182203 |
| 手部最大稳态跟踪误差/rad（界限0.02） | **0.21534176 FAIL** | 0.00422972 |
| 最大线性Mimic残差/rad（界限0.02） | 0.00068716 | 0.00049658 |
| 最大关节越限/rad（界限0.01） | 0.00012328 | 0 |
| 最大启用非相邻自接触交叠/mm（界限2） | **12.477555 FAIL** | 0 |
| 右手与外物接触步数 | 0（没有外物） | 128 |
| 最大外物交叠/mm（界限2） | 无外物 | 1.149328 |
| 机器人运行前后物理字段一致 | true | true |

首个失败为开合第229步、`t=0.458s`、`finger11/close_ramp`：`base_link↔finger12_link`未过滤凸包接触，交叠2.012186mm、法向力37.867N。首次稳态跟踪失败在第451步、`t=0.902s`。交叠超限共750步，稳态跟踪超限100步；动态最大交叠在第6511步、`finger12_link↔finger42_link`。同对最大法向力约66.6365N（第6498步）。这些是当前粗碰撞模型的求解结果，尚不能区分真实三角表面自碰撞与凸包填凹导致的额外交叠。

| 主动通道 | 独立闭合稳态峰误差/rad | 联合闭合稳态峰误差/rad |
|---|---:|---:|
| finger11 | **0.02406233 FAIL** | **0.04422274 FAIL** |
| finger12 | 0.00273065 | **0.09755731 FAIL** |
| finger21 | 0.00837540 | 0.00837581 |
| finger31 | 0.00841394 | **0.13297492 FAIL** |
| finger41 | 0.00839073 | **0.21534176 FAIL** |
| finger51 | 0.00834837 | 0.00834712 |

所有独立通道均产生响应并能释放；独立释放稳态误差峰低于0.00835rad，联合释放峰0.008647rad。5个从动关节在实际步进中保持线性耦合，但不能由此声称全行程跟踪通过。

探针Reset时与机器人最小距离为0.670076mm、无初始接触；仅初始夹具放置做几何选择，之后受重力演化。第8–135步观察到球与`gripper0_base_link`接触，最大法向力0.0806277N。**这只验证基本掌部外物接触，未证明指尖抓持、抬升或保持。** [探针初态](probe-placement.json)、[两段完整结果](run-result.json)。

9个静态姿态各显式检查1190组碰撞几何对，未发生距离查询错误；不经生产过滤的距离检查仍记录默认父子、同刚体及已有exclude原因。启用非相邻凸包最大交叠8.904728mm，默认过滤的几何候选最大34.738855mm；后者可能包含固定装配的凹面啮合，不计为已经证实的实物穿透。[静态距离记录](static-distance-audit.json)。未执行CAD三角面连续碰撞验证。右手抓持`UNKNOWN`，LIBERO `NOT_RUN`。

## 复核与交接

```bash
source examples/libero_reference/env/activate.sh
python examples/right_hand/run.py --output runs/right_hand_002
```

本次完整执行返回`2`（筛查FAIL/UNKNOWN），不是执行错误；`1`才表示运行错误。两段JSONL状态和接触均逐步保留，gzip内原文件SHA及行数见[artifact-index.json](artifact-index.json)。解压即可读取，无需旧工程。静态预览[right-open.png](right-open.png)仅由`mj_forward`生成，0物理步。

同一隔离环境运行`python -m unittest discover -s tests -v`（21项）、`python -m unittest discover -s examples/libero_reference -p 'test_*.py' -v`（47项）、`python examples/libero_reference/env/test_provenance.py`（3项），全部通过；`pip check`及提交文件SHA核验通过。原始日志位于本目录。软件测试通过不替代以上物理FAIL。

289个受保护左手文件哈希保持不变，原工作树HEAD及未提交README原样保留：[保护核验](left-protection.json)。右手本轮没有改动左手MJCF/Mesh、控制参数、D1/D2程序、fixture或验收证据。

此候选可审查与复现，**暂不能作为全幅开合已验收的右手稳定版**。下一步先审查拇指根部及联合闭合的碰撞几何，区分真实结构干涉和凸包近似误差，再单独决定是否需要源Mesh分块或其他模型修复；本轮未自动改代理、限位、接触参数、控制器或阈值。
