# R_close149_left_drive_profile_v1

Issue #5 [6296626303](https://github.com/LeoandDream/Nero_DH116_assets/issues/5#issuecomment-6296626303), baseline `24c0d44dcaab3eeac52d2b0c8f02dfd0fec15773`.
**PASS：源文件、既有编译快照及补丁的离线一致性。B 尚未编译、接入运行器或运行动力学，操作效果 UNKNOWN。**

## 唯一候选与参数来源

A 是已有 close149 真实运行配置。B 从该提交左手 `mjcf/nero_dh116.xml` 按同名关节提取以下原始字符串，不重新舍入；右手结构保持原值。

| 主动关节 | A kp / kv | B kp | B kv |
| --- | --- | --- | --- |
| finger11 | 50 / 2 | 0.65286574077483084 | 0.026114629630993234 |
| finger12 | 50 / 2 | 0.65991473097275011 | 0.026396589238910004 |
| finger21 | 50 / 2 | 0.5098435598707074 | 0.020393742394828294 |
| finger31 | 50 / 2 | 0.69307170971793874 | 0.027722868388717549 |
| finger41 | 50 / 2 | 0.51027003659620607 | 0.020410801463848244 |
| finger51 | 50 / 2 | 0.29451331029853522 | 0.011780532411941408 |

全部11个手部 hinge（11/12/13、21/22、31/32、41/42、51/52）：`damping 2→0`、`armature 0.01→0`；`frictionloss=0` 不变。共17个对象、34个属性，精确映射见 [profile.json](profile.json)，差异见 [model.patch](model.patch)。

这是预定义的多参数驱动配置包 A/B 对照。右手原值是上游仿真配置；左手最初调参推导未取得。不据此声称硬件标定正确、等带宽、等阻尼比或某单参数是唯一根因。

## 已核对与未验证

- [snapshot-audit.json](snapshot-audit.json)：按名称及 `actuator_trnid` 绑定核对右手源 XML、实际 `hand.xml`、保存的 formal/actual 编译快照，以及左手 XML/编译 fixture；86项检查通过、无冲突。覆盖 gain/bias、gear、ctrlrange、执行器/关节力限幅、damping/armature/frictionloss。
- 已保存快照中，执行器和关节两层力限幅标志均为0；对应 `[0,0]` 范围表示限幅未启用，不能解释成零输出。保留原角度目标范围与单位 gear，不新增限力。
- [runtime-audit.json](runtime-audit.json)：检查 close149→LIBERO 适配→模型拆分→stock robosuite 1.4.1 源码，未发现所查手部字段的运行时覆盖。NERO JPC 的 kp/kd 与手部伺服 kp/kv 分开处理。
- [offline-check.json](offline-check.json)：临时 XML 副本应用补丁成功；XML 语义差异恰为34个允许属性，恢复这34个 token 后与原 XML 字节一致。预测的 B gain/bias/DOF 数组明确标为 `PREDICTED_NOT_COMPILED`。[MuJoCo position 定义](https://mujoco.readthedocs.io/en/3.2.3/XMLreference.html#actuator-position)对应 `gainprm=[kp,0,…]`、`biasprm=[0,-kp,-kv,0,…]`。
- 81手部、306腕部、右手 body 质量/完整惯性/关节轴/限位、线性 Mimic、几何/接触/过滤、NERO、Reset 和求解器均未改。原始260条 close149 动作、每宏25步、2ms、RK4、最多6500步及 V1 规则保持不变；保留 lift 首条重复起点，机械臂目标首次变化仍在4776步。
- 保存的 post-step q/qvel 与 RK4 末内部阶段的力缓存不精确对时，未用其构造同一时刻力矩恒等式。历史早停及全部 FAIL 保留。

## 补丁复核与后续接入边界

这里只交付属性清单与补丁，不复制完整模型/资源，也不改旧 runner。`model.patch` 的目标是原 `finger22_local/model.xml` 的**副本**；资源解析必须保持原 `finger22_local` 上下文（含局部 `meshes/`），不能把本目录当作模型资源根目录。

从仓库根目录可仅用 Python 标准库和 Git 复核源 SHA 与补丁（仅在临时目录写 XML，不加载物理引擎）：

```bash
python3 - <<'PY'
import hashlib, json, pathlib, subprocess, tempfile
root = pathlib.Path.cwd()
p = root / 'examples/right_hand/candidates/left_drive_profile_v1'
d = json.loads((p / 'profile.json').read_text())
sha = lambda x: hashlib.sha256(x.read_bytes()).hexdigest()
for rel, expected in d['source_sha256'].items():
    assert sha(root / rel) == expected, rel
with tempfile.TemporaryDirectory(prefix='dh116-drive-offline-') as tmp:
    xml = pathlib.Path(tmp) / 'model.xml'
    xml.write_bytes((root / d['base_model']).read_bytes())
    subprocess.run(['git', 'apply', '--check', str(p / 'model.patch')], cwd=tmp, check=True)
    subprocess.run(['git', 'apply', str(p / 'model.patch')], cwd=tmp, check=True)
    assert sha(xml) == d['candidate_xml_sha256_in_original_base_context']
print('PASS: pinned sources and temporary patch application; no compilation or dynamics')
PY
```

动作文件 SHA256：`019173fae82db3b0b30b58572fb64e0c3ff400f0ad98912ff17c41dc69b1ecbf`；补丁后临时 XML SHA256：`e24d17904b9b1d4a55ae031a08bb1457f0f46c4170f987767d0acd161eb3e84b`。

现有运行器有原始配置精确相等断言，**本候选尚不具备直接运行条件**。后续若获授权，需要显式接入上述34属性的允许差异并核验实际编译结果，其余检查不变。本轮不实施该适配或启动 B。B 从完整 Reset 即改变驱动，不应要求前4750步物理轨迹等于 A；仅接触力降低而未形成有效抓持不能判为改善。A 复用已有真实记录，无需重跑。
