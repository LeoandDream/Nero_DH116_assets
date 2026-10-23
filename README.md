# NERO × DH116 — 四套研究级 MuJoCo 资产

四套研究级 MuJoCo 资产保持既有机械与控制接口。左手组合默认采用经5750步白碗验证的306块高细度腕部；原左手V1（15腕）继续保留为 `mjcf/nero_dh116.xml`，支持历史复现。

| 入口名 | MJCF | state / action |
| --- | --- | --- |
| `dh116_left` | [独立DH116-L](mjcf/dh116_left.xml) | 11D / 6D |
| `dh116_right` | [独立DH116-R](mjcf/dh116_right.xml) | 11D / 6D |
| `nero_dh116_left` | [NERO＋DH116-L（306腕）](mjcf/nero_dh116_left_fine.xml) | 18D / 13D |
| `nero_dh116_right` | [NERO＋DH116-R研究配置](mjcf/nero_dh116_right_research.xml) | 18D / 13D |

两套独立手与右手组合保持原字节；左手新默认仅替换NERO link5＋法兰碰撞为306块，手部76块、驱动与Mimic不变。右手仍为 **B＋functional_v1、81手部＋306腕部配置**，306并不代表全域最优。详见[左腕升级说明](docs/LEFT_WRIST_FINE.md)与[历史交付报告](docs/DELIVERY.md)。

## 安装与最小控制

Linux、Python3.11、MuJoCo3.2.3、NumPy1.26.4；组合控制示例额外使用stock robosuite1.4.1。四模型加载及此控制入口不需要LIBERO或历史实验目录。

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r examples/requirements-control.txt
python examples/control_asset.py --help
python examples/control_asset.py --asset dh116_left
python examples/control_asset.py --asset dh116_right
python examples/control_asset.py --asset nero_dh116_left
python examples/control_asset.py --asset nero_dh116_right
```

`--asset`命令从模型参考keyframe初始化，默认执行250物理步以保持参考目标。复制模型时同时保留相对引用的 `meshes/`；独立手掌部固定在局部原点，组合保留原装配与世界基座位置。查看四入口及资源闭包：[docs/assets.json](docs/assets.json)。

```python
from examples.control_asset import AssetRuntime
robot = AssetRuntime("dh116_right")
command = robot.home_action()
command[-6:] += 0.1  # 小幅手目标；各关节行程从所选模型读取
robot.set_action(command)
robot.step(25)       # 2ms物理步；常用20Hz命令保持25步
print(robot.state())
```

## 统一接口

- 独立手状态为11关节位置rad；主动动作顺序为 `finger11, finger12, finger21, finger31, finger41, finger51` 六个归一化绝对目标 `[-1,1]`。完整关节名称、顺序和维度见索引；左右机械、轴、惯性、限位及必要驱动差异保留。
- 组合状态是NERO7＋手11；动作是NERO7个绝对目标角度rad＋手6。独立手不补齐不存在的NERO状态。
- MuJoCo `ctrl` 是独立手6个目标角；组合是7个电机力矩＋6个目标角。**13D位置action不能直接写入ctrl。** 示例复用stock JPC绝对位置入口和原手部position servo、线性Mimic。
- 独立手保留已验收的五组显式掌部父子过滤，以恢复固定到world前的原接触资格；本轮不新增过滤或改变物理字段。

## 检查与使用边界

```bash
python -m unittest discover -s tests -p test_four_assets.py -v
sha256sum -c SHA256SUMS
```

上述交付检查仅解析XML、校验资源/接口与哈希，不编译或运行MuJoCo。本轮不重跑仿真；动力学能力引用冻结版本已保存并认可的证据。原始白碗指标只支持固定工况的研究使用，不能当作全行程、任意物体、完整LIBERO任务或实物标定通过。

原[左手V1及LIBERO参考入口](examples/libero_reference/README.md)保持原样，其冻结复现仍加载 `mjcf/nero_dh116.xml`（15腕），不会跟随默认索引切换。右手及其他历史实验/FAIL原始证据保留于固定GitHub提交。

## 来源与许可

NERO与经核对的厂家法兰源件/衍生物采用MIT，DH116采用Apache-2.0。厂家原版权/许可通知、法兰逐项再分发依据见[来源与许可](licenses/SOURCES.md)及[法兰来源证书](licenses/flange-provenance.json)。既有仓库公开不作为授权依据。

本次默认左腕切换为独立审查变更；旧研究基线及历史PR保留，未经复核不自动合并或发布Release。
