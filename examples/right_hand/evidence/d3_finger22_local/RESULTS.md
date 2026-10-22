# finger22 四片候选：静态接触改善，数值验收仍阻塞

任务：[PR #4 / 6271816822](https://github.com/LeoandDream/Nero_DH116_assets/pull/4#issuecomment-6271816822)。基线 `360e53a023624e96bd88aef549927655b5034445`。**仅构造一版；总体 `FAIL_BLOCKED`，未进行新动力学，未获交付批准。**

| 项目 | 实际结果及范围 |
|---|---|
| 候选与冻结 | **PASS**。`../../candidates/finger22_local/model.xml` SHA256 `7f9d84061c3e7d176763c688d2389f8b3d3e38386b8e990a0fa047ff2bc3329c`。同 body 的原单 geom 被四片替换，无叠加；手部 78→81，其余77块、306腕部、全部旧资源字节、装配/质量/惯性/关节/控制/Mimic/过滤及物理选项不变。`input-audit.json` 核查独立机器人与原生场景；426个非目标旧 mesh_graph 一致。已移除目标的旧未使用资产 graph 差异另列，不隐去。 |
| 源面裁剪 | **PASS**。唯一切位 local Z=9.8/23.3/32.0mm；16784个厂家源三角面逐面精确裁剪、跨面保留、共面唯一归属。面积 `0.003240273332582338m²` 守恒；raw 连续未覆盖面积0。面ID、映射和切位先于凸化写入候选 `registered.json`。 |
| 4741/4742 RK4 | **PASS，表面/凸体静态范围**。使用原缓存 finger22 与碗 body 姿态，36个原生碗 box ×4片均分离。g22源面间隙分别 **0.345218594 / 0.329680456mm**；四片对g22的最小投影分离下界分别 **0.286671920 / 0.270602903mm**。g23亦分离，见 `rk4-witnesses.json.gz`。没有用post-FK冒充缓存。 |
| 全742 close状态 | **PASS，固定保存姿态的接触表面回归**。步4001–4742，507个合资格其他geom/状态，共376194个目标对子状态；375768个全几何AABB分离，426个完成窄相及源面核验。原严格交叠158→新0；80个状态的原生finger22接触集合改变。未发现源面相交被新代理漏检，也无新增目标交叠。非finger22的926个原生手—碗接触对子状态及距离逐项相同，包括finger13/finger14/finger23；这不等于其全部厂家几何已获重新认证，更不预测重跑接触力。 |
| 编译数值 | **FAIL，保持1e-10m原标准**。compiled 源面覆盖有3602个三角面超出数值容差，未覆盖源面面积 `0.0002531098311066771m²`；这是贴面纳米偏移形成的面积统计，不是删除这么大的实体面。对厂家凸参考最大外侧半空间偏差 **0.967846nm**；raw↔compiled支撑顶点双向最近距离最大 **1.842606nm**。不能据此豁免旧FAIL。 |
| 接缝与保真边界 | raw三切缝外露共面封盖面积均0；compiled前两缝测得 `2.86057e-14 / 5.60074e-17m²` 的共面外露面积，第三缝0；倾斜微缝仍由覆盖/距离检查约束。失败朝向+X源面6165/6176固定1024采样的外边界偏差最大 **0.092698mm**。全表面4096采样显示源面埋入最大 **6.335794mm**、反向边界距离最大raw **3.079960mm** / compiled **3.025393mm**，双向急角法向差最大90°；孔槽/圆角处不应宣称全表面保真。厂家与原交付raw的全顶点双向观测偏差最大 **0.263403mm**。这些均为有限观测，不是全局Hausdorff界。 |
| 实体与动态 | **UNKNOWN**。厂家STL有81条非流形边；源面覆盖不证明铰孔/凹腔空区保持，更不证明实体等价。4518原正力接触是finger13，缺少finger22 RK4缓存，只作post-FK比较。共1484次静态`mj_collision`、1484次FK，**0积分、0forward、0求解、0新力/抬升/持握结果**。 |

证据：`summary.json`、`scan-summary.json`、`close-states.json.gz`（全部742状态、原/新接触集、所有窄相与源面见证）、`raw/compiled-coverage.json.gz`、`raw/compiled-quality.json.gz`、`quality-summary.json.gz`、`working-face-patch.json`、`api-audit.json`。LP证书只稀疏存储非零对偶项，原始实际轨迹未改；全部状态/分类、原始与新接触距离及失败信息保留，完整原始半空间可由冻结编译网格重建。

复核（仓库根目录；已有锁定环境，MuJoCo 3.2.3；使用**新的输出目录**）：

```bash
env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data_robot/verification/nero_dh116_right/.reference-env/bin/python -B tests/test_finger22_local.py
env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data_robot/verification/nero_dh116_right/.reference-env/bin/python -B examples/right_hand/finger22_static.py \
  --output /tmp/dh116-finger22-static-review \
  --libero-assets /data_robot/verification/nero_dh116_right/.reference-deps/LIBERO/libero/libero/assets
sha256sum -c examples/right_hand/evidence/d3_finger22_local/SHA256SUMS
```

`tools/build_finger22_local.py` 是本次唯一构造入口，拒绝覆盖现有候选；复核无需再次构造。原型检查器曾补齐漏报统计并中止一次耗时的旧参考交集附查，未改变候选、切位或阈值；完整compiled覆盖和742状态回归已经实际完成。

**唯一下一步：提交本版静态阻塞复审，决定编译数值FAIL及孔腔近似的最小处理方案；当前不批准动力学或发布，不自动再分片、不改阈值。** 左手V1、正式左右手、共享资源、全部历史FAIL、main与用户README均保留。
