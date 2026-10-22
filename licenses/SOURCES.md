# 来源与许可

- NERO URDF：agx_arm_urdf，commit 1ee5659d02b33b9379fd647a2e5647800d67f0f4；随源MIT许可。
- DH116左手：sorrowfeng/leadtron_hand_descriptions，commit 14275cfe32cce1fe13a16df712e8a0e610a0227e；随源Apache-2.0许可。
- 厂商连接件：agilexrobotics/agx_arm_sim，commit f8cd8b147c75d59e14f90fb0646770eefa268ed0；未发现独立许可，来源缺项保持，不用其他许可代替。
- 衍生碰撞代理和格式转换：本项目转换产物，来源与SHA见SOURCE_MANIFEST.json及URDF_SOURCES.json。源仓库许可文本随包保留。

## 四入口研究交付补充

- DH116右手同样来自 `sorrowfeng/leadtron_hand_descriptions@14275cfe32cce1fe13a16df712e8a0e610a0227e` 的 `DH116-R000-A1`；厂家源文件及逐文件SHA见 `RIGHT_SOURCE_MANIFEST.json`，Apache-2.0文本随包保留。
- 新独立手从各自组合的真实手部子树提取；右手组合从 `7998718` 保存的 B 成功模型打包。没有重新生成几何。`docs/assets.json:resources` 为每份资源记录原仓库路径、目标路径、SHA及原字节复用/复制操作。
- `meshes/dh116_right/research/` 来源为既有右手候选；`meshes/nero/research_wrist/` 是已保存的306块腕部工程候选原字节副本，溯源继续指向原候选及其来源清单，并不重新授权厂家表面或改变原许可。
- 独立手MJCF仅引用DH116资源，不含NERO或法兰。两套组合含厂家法兰；其独立许可缺项继续阻止无保留的公开授权发布，不以研究验收通过替代许可确认。当前仅提交审查候选，不发布Release。
