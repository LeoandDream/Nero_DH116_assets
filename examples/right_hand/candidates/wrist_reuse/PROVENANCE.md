# DH116整部件复用诊断候选

授权：PR #4评论6268804256及用户执行提示。装配基线18497ba中的finger13_local（手部78块）。

只从SHA256 `801e5d24cbace320511dcfb25afdce11c8ed2f4afd7572dd1007145d21ea500a` 的 `nero-engineering-static-20261019.zip` 原字节复制306份OBJ。`manifest.json`绑定每份来源、SHA、父body及替换的旧15个geom。link5原7→147，法兰原8→159；没有叠加旧代理，没有顶点变换、缩放或重新分解。既有body装配保留，所有新增geom的XML位姿为零，不能再施加Gripper法兰变换。

其余资源使用checkout内相对路径，物理字段、手部78、控制/Mimic/过滤和动作均保留DH116基线。**SOURCE_PROVENANCE.md与SOURCE_UPSTREAM_README.md是原ZIP来源材料，提及的kp80和Gripper验收不是本DH116候选的配置或许可。** 本候选仍为七个力矩motor、六个kp50/kv2手部position；不得替换正式模型。

上游agilexrobotics/agx_arm_sim@f8cd8b147c75d59e14f90fb0646770eefa268ed0；source link5 STL SHA256 `dc497f5a9bab2d0d314250134d4ec86617abea7205d122db41b765acdb86ae7e`，法兰 `186697a73e7cafd9fd41d96f1eea9b34b444c058b9561fb2e3ea439c835ae2e8`，与本仓库源件逐字节一致。保留原README的MIT声明；原树缺少完整独立许可通知，正式Release前仍须补齐，没有新增授权声明。

风险及复核命令见 `../../evidence/d3_wrist_reuse/RESULTS.md`。125正J6边界状态、0.300mm根部近似、2.17mm外填及原1e-10m问题均未获DH116豁免。离散静态分离不证明动态跟踪、抓持、完整关节域或正式发布。
