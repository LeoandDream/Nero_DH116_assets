# 科研接触停止指标 V1：仅离线，尚未批准

基线 `54b5a480221b8f86c15d33dbe84bad397e960f3a`，[任务6291982663](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6291982663)。**唯一推荐候选 `DH116_RESEARCH_CONTACT_V1 / RESEARCH_ONLY_NOT_APPROVED`**，规则 SHA256：

`48632dfa4bbe0630ef7d61b9e706240d7769ad6361d588e1d02025bde4bc6454`

**目标。** 原2mm原生MPR深度不是厂家材料变形真值，不能单独解释损坏风险；它仍是正式门槛。本提案仅对 close/lift/hold 中所有合资格手部 link↔白碗接触，增加时间、载荷及发展趋势判断；不按finger23、g23或4769开例外。其他机器人自碰、桌面及其他外物仍沿原2mm即时停止。

## 冻结的一套规则

对每个无序geom对和body对同时跟踪：`d=max(0,-contact.dist)`，`F=Σmax(Fnormal,0)`。每个组内所有原生接触点只计一次；body聚合防止g22/g23或代理块切换清空计时。F是标量载荷和，**不是净合力**，仍有接触表示依赖。

固有尺度来自已保存编译快照：手质量 **0.23767046kg**，`F*=m_hand·g=2.3315472126N`；手/碗等优先级、solmix=1，混合 `τ=(0.020+0.001)/2=10.5ms`，dt=2ms。向上取整：短窗 **12ms（6间隔/7样本）**，长窗 **32ms（16间隔/17样本）**。solref只是受阻抗影响的参考约束时间尺度，不能把3τ称为精确恢复时间；[固定3.2.3文档](https://mujoco.readthedocs.io/en/3.2.3/modeling.html#solver-parameters)。

| 级别 | 可执行条件 |
| --- | --- |
| 记录/预警 | 任一组 `d>2mm`；或 `F≥F*=2.331547N`。同时记录窗口深度/力净变化，不把MPR中点当源面。 |
| 立即停止 | 原数值/时钟、臂跟踪、仅hold主动跟踪、Mimic、限位、腕部意外接触及路径包络保护全部保留；非适用域接触 `d>2mm`；适用域任一组 `F≥10F*=23.315472N`。缺步/重复步、身份或缓存时序不符、非有限接触数据则 `LOG_INVALID_UNKNOWN`，不能放行。 |
| 持续异常停止 | **任一条件成立即停：**①`d>2mm`逐样本连续首末跨度≥32ms，零力也不豁免；②连续深接触≥12ms，短窗平均采样载荷≥F*，且该窗深度末值>首值（窗口净增长，非逐帧单调）；③完整32ms窗采样载荷暴露≥`3F*×32ms=0.2238285324N·s`，无须先超过2mm。 |

时间是**已观察缓存首末跨度**，首次深接触样本时长为0；不能证明相邻缓存之间无间断。实际观察到一帧无接触时，深接触计时清零，滚动载荷历史保留。载荷暴露是保存末RK4阶段力的**梯形样本统计**；没有所有内部阶段的力/权重，**真实积分冲量UNKNOWN**。比较使用样本binary64值的有理数运算，不额外加浮点容差。[contactForce定义](https://mujoco.readthedocs.io/en/3.2.3/APIreference/APIfunctions.html#mj-contactforce)。

**选值依据与限制：**2mm未提高；1τ/3τ区分瞬时观察与未消退接触。1/3/10倍整手重力只是一套透明、**未经标定的研究运行保险**，没有厂家材料/损伤或允许夹持力依据，不能称为安全限值。规则SHA在本次首次接触序列影子评分前冻结，此后未按结果调值；但4769结果原本已知，仍属后验设计，不是独立前瞻验证。

## 4769步影子回放

对照270条冻结日程逐步核对已执行的4769步及原健康指标；实际仅执行到close第769/1000步。旧协议首次停止仍为 **4769 / ROBOT_CONTACT_OVERLAP**。推荐规则首次载荷预警 **4723**，共46个样本步；首次深度预警 **4769**，1个样本、已观察深超限跨度 **0ms**。**推荐版在已保存前缀内无停止，不表示真实实验通过。**

| finger23↔白碗body聚合 | 4731 | 4742 | 4768 | 4769 |
| --- | ---: | ---: | ---: | ---: |
| 最大原生深度 mm | 1.159160 | 1.451230 | 1.946405 | 2.029398 |
| 全部接触点法向载荷 N | 3.101157 | 3.498528 | 4.502273 | **4.555848** |
| 最近32ms采样载荷暴露 N·s | 0.080567 | 0.103459 | 0.140849 | **0.141068** |

4518首次正力手碗接触（finger13）；finger23在4683首次接触，末步连续缓存跨度172ms，但超过2mm只有最后一帧。4769单个g23接触Fn=1.737958N并非整指载荷。其余独立即时保护无触发：臂最大误差0.088806594rad、Mimic0.000278909rad、越限0、warning0；close中的主动误差0.046661215rad不误套hold门槛。

**预定单因素敏感性，未据此重选规则：**长窗2τ/4τ（22/42ms）均无前缀停止；载荷尺度×2也无停止；**载荷尺度×0.5在4751首次因载荷暴露停止**（当时body载荷4.362913N、深度1.684671mm、32ms暴露0.112549556N·s）。该变体4752–4769只是旧数据的反事实尾部。结果对未标定载荷尺度敏感，不能声称风险已被物理校准。

**唯一下一步：**提交这套冻结规则及载荷尺度敏感性供审阅。若接受其明确的科研保险边界，值得另行申请一次限定探索；当前状态仍未批准，不能直接开跑。4769之后的close/lift/hold均UNKNOWN，不能推断会成功。

## 证据与执行范围

[research-policy.json](research-policy.json)、[shadow.py](shadow.py)、[test_shadow.py](test_shadow.py)、[summary.json](summary.json)、[逐步影子记录](shadow.jsonl.gz)、[verification.json](verification.json)。仅新增本目录；没有复制原轨迹或生成几何。**26项判据/原保护边界测试通过**，两次纯离线输出逐字节相同；1212个旧跟踪文件及用户README保持SHA不变。

执行使用 Python3.14.7 **标准库**，无MuJoCo/robosuite/LIBERO导入、模型编译、Reset、forward、求解或积分，未接入任何线上runner；81块手部、306块腕部及全部原配置未改。前轮3次编译自检的约束FAIL及其披露原字节保留；旧2mm FAIL、正式0.1nm FAIL和高保真未决均不回写，PR保持Draft、不合并。

从仓库根目录复核，无须任何仿真依赖：

```sh
sha256sum -c examples/right_hand/evidence/d3_research_contact_policy/research-policy.sha256
python -B examples/right_hand/evidence/d3_research_contact_policy/test_shadow.py
python -B examples/right_hand/evidence/d3_research_contact_policy/shadow.py --output runs/research-shadow-new
```
