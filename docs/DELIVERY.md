# 四资产精简交付

**基线：main@9b5ee0d。模型冻结：5fc2ea3。** 本分支直接从main选择性加入交付文件，不以PR #4或PR #7为Git父提交，不批量引入其候选模型、调参工具、缓存、视频、轨迹或研究报告。main原有左手V1和既有LIBERO参考文件保留。

## 保留的交付内容

- 四份稳定MJCF、506份声明所需的原字节运行资源；复制时不裁剪XML声明、重新分块、换网格或改变物理字段。
- `examples/control_asset.py`统一入口与最小依赖文件；原手部position servo及stock robosuite1.4.1 JPC不变。独立手11D/6D、组合18D/13D，共用顺序和单位，保留真实机械差异。
- `docs/assets.json`资源及来源索引、`SHA256SUMS`、第三方原许可、法兰来源证据和7项纯静态测试。
- 本页与机器可读[交付检查](delivery-check.json)。研究历史通过下方固定提交链接访问，不成为运行时依赖。

| 入口 | 冻结验收状态 | 本次整理 |
| --- | --- | --- |
| `mjcf/dh116_left.xml` | 2400步有界六通道控制认可 | XML及资源原字节相同 |
| `mjcf/dh116_right.xml` | 右手机械＋B驱动，2400步有界控制认可 | 同上 |
| `mjcf/nero_dh116.xml` | 左手V1、400步组合控制及原白碗证据 | main原文件不覆盖、原字节不变 |
| `mjcf/nero_dh116_right_research.xml` | 成功B＋functional_v1配置，400步组合控制及原白碗证据 | XML及资源原字节相同 |

## 本次检查：静态一致性，不新增仿真

实际比较了513份模型/资源/运行文件与冻结Git对象的完整字节，没有差异。将最小闭包复制到全新空目录，仅包含四模型、资源、接口、依赖、来源许可及静态测试，不带任何历史实验或LIBERO目录；7项测试通过。最小验证包521文件，约71.49MiB。控制CLI的 `--help`通过，未导入MuJoCo或robosuite。

没有模型编译、Reset、forward、mj_step、调参或新抓持尝试。运行功能沿用已认可的同字节资产证据；本次静态检查不冒称新的动力学结果，也不改变原实验阈值。

```bash
python -m unittest discover -s tests -p test_four_assets.py -v
sha256sum -c SHA256SUMS
```

## 法兰公开再分发依据

本轮补齐此前缺失的许可溯源：在厂家固定 `agx_arm_urdf@1ee5659` 中找到完整MIT许可及法兰STL/DAE；源件与本项目原件逐字节相同。原视觉转换记录的源/OBJ/纯色贴图SHA，及左8块、右159块碰撞代理SHA均能对应。随包厂家原版权/许可通知与上游相同。完整来源、固定链接、Git blob、SHA及派生映射见 [licenses/SOURCES.md](../licenses/SOURCES.md)和[flange-provenance.json](../licenses/flange-provenance.json)。

因此所列素材的MIT再分发依据已补齐，不再仅凭“仓库公开”或SIM仓库缺少正文的MIT声明推断授权；保留上游通知是再分发条件。不扩展为对其他未核对SIM素材的许可结论。原冻结/历史文档的UNKNOWN不回写；本页记录新查到的证据。

## 已认可的原始证据及边界

- [Issue #6状态记录](https://github.com/LeoandDream/Nero_DH116_assets/issues/6#issuecomment-6350498728)及[PR #7交付审查](https://github.com/LeoandDream/Nero_DH116_assets/pull/7#pullrequestreview-5620312019)：四套资产研究级技术验收通过。
- [5fc2ea3原四资产报告及逐步控制记录](https://github.com/LeoandDream/Nero_DH116_assets/blob/5fc2ea346ba7bccb80052024d9c16c3ee2b5a526/examples/evidence/four_assets/RESULTS.md)。首版固定掌部的FAIL及后续接触资格修复证据保留。
- [右手B＋functional_v1成功记录](https://github.com/LeoandDream/Nero_DH116_assets/blob/5fc2ea346ba7bccb80052024d9c16c3ee2b5a526/examples/libero_eval/evidence/functional_v1/RESULTS.md)：6500步，沉降基准约100.494mm抬升、1.5s持握。
- [左手原白碗记录](https://github.com/LeoandDream/Nero_DH116_assets/blob/5fc2ea346ba7bccb80052024d9c16c3ee2b5a526/examples/libero_reference/evidence/RESULTS.md)：5750步，沉降基准约103.95mm抬升、1.5s持握。

历史全幅FAIL、数值附查FAIL和高保真UNKNOWN不变；固定白碗能力不等于任意物体、全行程、实物校准或完整LIBERO柜顶任务成功。独立手没有独立白碗任务成绩。

本分支仅供交付审查：可直接对main审查差异，无需先全量合并PR #4/#7；不自动获得合并或Release授权。PR #4/#7仍保持Draft，用户主工作树及README不变。后续科研实验与本次交付分别管理。
