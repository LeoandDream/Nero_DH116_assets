# 来源与许可

本页记录四资产交付所用第三方素材的许可及派生来源；不能因GitHub仓库公开就推定允许再分发。

| 素材 | 固定上游 | 许可及通知 |
| --- | --- | --- |
| NERO源模型及派生网格 | `agilexrobotics/agx_arm_urdf@1ee5659d02b33b9379fd647a2e5647800d67f0f4` | [厂家MIT原文](NERO_URDF_MIT.txt)；Copyright (c) 2026 aalicecc |
| DH116-L/R源模型及派生网格 | `sorrowfeng/leadtron_hand_descriptions@14275cfe32cce1fe13a16df712e8a0e610a0227e` | [Apache-2.0原文](DH116_APACHE_2_0.txt) |
| 厂家法兰源STL/DAE、视觉转换及碰撞派生 | 同一固定 `agx_arm_urdf` 中的 `nero/meshes/gripper_flange.stl`、`nero/meshes/dae/gripper_flange.dae` | 同上厂家MIT通知；逐项来源和哈希见 [flange-provenance.json](flange-provenance.json) |

## 法兰公开再分发核对

此前记录“许可待确认”，依据是 `agx_arm_sim@f8cd8b147c75d59e14f90fb0646770eefa268ed0` 虽在README声明MIT，却没有独立LICENSE正文。本轮补查了它所引用的固定URDF子模块，找到实际MIT许可文件；不是给无许可素材自行补写授权。

- [厂家MIT LICENSE](https://github.com/agilexrobotics/agx_arm_urdf/blob/1ee5659d02b33b9379fd647a2e5647800d67f0f4/LICENSE)与随包 `NERO_URDF_MIT.txt` 的Git blob均为 `601f56ad41b103897cd4227a2d1ea0594b389d08`。厂家[README](https://github.com/agilexrobotics/agx_arm_urdf/blob/1ee5659d02b33b9379fd647a2e5647800d67f0f4/README.md)明确包含URDF与3D网格资源，并链接该MIT许可；未发现针对这两份法兰源件的例外声明。
- 本仓库法兰源STL/DAE分别与上述固定MIT仓库中的blob `b15a867f76655e5502772486fe4028a2baaaf576` / `e0885e73a999c63f7b726648d1f5e721ebd68331`逐字节一致。
- 原视觉转换记录将同一DAE的SHA绑定到实际视觉OBJ和1×1黑色RGBA材质贴图的SHA；贴图来自源diffuse颜色，不是新增第三方图片。
- 左侧8份和右侧159份法兰代理的实际文件SHA，与原STL来源/派生记录一致。右侧来源记录保留原ZIP及每份网格的SHA，不将旧Gripper控制参数或验收豁免导入DH116。

据此，所列法兰源件及记录中的衍生物具备MIT再分发依据，副本及实质性衍生物继续附带厂家原版权和许可通知。本结论仅覆盖有记录绑定的所列文件，不给 `agx_arm_sim` 的任意其他素材作整体授权推断。

## 资源与证据

`docs/assets.json` 列出四模型、运行资源及SHA。资源条目中的 `origin` 属于冻结提交 `5fc2ea3` 的历史路径，可通过同文件的 `resource_origin_context` 定位；不是运行时文件依赖。左手原提取来源保留在 `SOURCE_MANIFEST.json`、`URDF_SOURCES.json`；右手来源和原验收结果通过精简交付报告中的固定提交链接查阅，不复制历史研究目录。

本轮只新增来源/许可证据，没有重新生成网格、改变模型或回写历史许可UNKNOWN、FAIL或实验结论。研究级技术验收认可不等于允许直接合并或发布Release。
