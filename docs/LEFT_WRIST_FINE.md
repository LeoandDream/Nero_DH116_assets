# 左手组合默认腕部升级：15 → 306块

本变更将 `docs/assets.json` 的 `nero_dh116_left` 指向 `mjcf/nero_dh116_left_fine.xml`，其中NERO link5有147块、末端法兰159块；左手76块手部碰撞代理、驱动、Mimic、NERO机械字段与动作协议均未改动。另两种独立手和右手组合均不改变。

## 保留原有基线
原左手V1 `mjcf/nero_dh116.xml` 的字节保持不变，原 `examples/libero_reference/` 冻结运行仍使用V1，因此历史5750步和SHA复现不受影响。通过 `examples/control_asset.py --asset nero_dh116_left` 调用的新默认组合才使用306腕。

## 候选来源与已完成验证
左306网格来自 [PR #11重执行的源几何分区产物](https://github.com/LeoandDream/Nero_DH116_assets/blob/3a1687c579906f33ce5612de730f0b9cb8394c52/examples/wrist_study/README.md)，不是复制右侧交付OBJ。本次复制其相同306份Git blob到稳定路径，仅更改MJCF标签与资源相对路径（源候选SHA256 `dedec2c3dcb4f05f9799c5c8149f023a82e103c5718aeeed4b8cb1028540e7d6`；交付候选SHA256 `f4393af4e672dc9fd8c8fb87a917977521caa017118ab13120f35219b505a012`）。

既有左306候选在原生5750步白碗任务中完成相对沉降抬升103.952mm、无环境支撑对向持握1.5秒；状态/动作原始JSONL与旧左手V1历史轨迹逐字节相同。在同手别和同姿态短窗口测试中，306代理的 `mj_step` 耗时高于15代理约9%–19%。目前没有发现左手当前白碗任务的功能收益；采用306是为了减少其他可能轨迹下的腕部虚假接触风险，并非宣称所有任务更好。

[回归和局限记录](https://github.com/LeoandDream/Nero_DH116_assets/blob/3a1687c579906f33ce5612de730f0b9cb8394c52/examples/wrist_study/evidence/review.json)保留已知约0.300mm根部近似、约2.17mm局部外填风险。未做全域高保真或实物标定认证，原生柜顶放置尚未成功。

## 交付后复核
必须在全新环境中验证 `python -m unittest discover -s tests -v`、`sha256sum -c SHA256SUMS`、四套资产最小加载与有界控制。应核对新交付模型的编译物理字段与PR #11原模型仅存在模型标签和路径元数据变化；未核对之前不得将既有5750步成功结果当成新封装版新动力学实验。

资源源头为NERO/法兰MIT与DH116 Apache-2.0；原厂通知随包保留，详见 `licenses/SOURCES.md`。
