# D2 最小修复与固定参考回归

依据 [PR #2 D2评论](https://github.com/LeoandDream/Nero_DH116_assets/pull/2#issuecomment-6221968815)，执行代码为 `68057c2d08492ec3c23ad93bd6ded298942927ab`，基线为 `ec356b9f5cac37cf0d1dd4dce150aee9b2404e3d`。后续提交仅补充本次证据、链接与SHA清单。

本次仅修复完整JSON双向比较和通过退出码的条件，并澄清声明字段。MJCF/Mesh、驱动、接触、Reset、动作生成、控制适配、停止规则、固定依赖、物理/控制器/Reset验收夹具和原验收数值均未改变。fixture manifest仅刷新4个修改过的代码/测试哈希；原D1-B证据保持原样。

## 修复结果

- 所有顶层及嵌套字典字段双向比较，新增/缺失字段均报告差异，数字逐值精确比较，无浮点容差。
- `historical_trajectory_equal`表示完整字段/数值一致；`raw_jsonl_bytes_equal`单独表示解压原始JSONL字节SHA一致。空白、键顺序或数字写法改变时可出现前者true、后者false。
- runner仅在两者均true、230命令、5750步且`DIAGNOSTIC_FINISHED`时返回`EXACT_REFERENCE_REPRODUCED`/0；差异为2、运行错误为1。独立比较CLI任一比较不一致返回2。
- `native_task_result=false`来自真实`env.check_success()`。`strict_grasp_success_claim=false`保留兼容，仅表示未提出严格成功声明；没有运行或新增额外严格抓取谓词。

## 实际执行与退出码

复用已验证的独立固定环境，在 `/data_robot/verification/nero_dh116_d1b` 中执行；整个 `/data_robot/projects` 与旧 `/tmp` 被空tmpfs隐藏。完整命令见 [validate.sh](validate.sh)，挂载方式与[D1-B隔离验收](../RESULTS.md)一致。[isolation.json](isolation.json)记录开始状态；[environment.json](environment.json)记录此次版本、导入路径和stock文件校验。

| 命令 | 退出码／实际结果 |
| --- | --- |
| `python -m unittest discover -s examples/libero_reference -p 'test_*.py' -v` | 0；47项通过，含新增顶层/嵌套新增缺失/数字/结构/同义异字节输入的标志与退出码检查 |
| `python -m unittest discover -s tests -v` | 0；1项通过 |
| `python examples/libero_reference/env/test_provenance.py` | 0；3项通过 |
| `python examples/libero_reference/env/provenance.py --output runs/d2/environment.json` | 0；固定环境、1019项stock及1116项LIBERO文件、EGL通过 |
| `python -m pip check`、`sha256sum --check --status SHA256SUMS` | 均0 |
| `python -u examples/libero_reference/run.py --output runs/d2/reference_001` | **0；230命令／5750真实步；5874项预检无差异** |

[execution.json](execution.json)记录shell实际返回码；[run-result.json](run-result.json)、[comparison.json](comparison.json)、[metrics.json](metrics.json)为运行原文。对应测试日志随本目录保存。

| 关键结果 | 实际值 |
| --- | --- |
| 字段/数值严格相等、原始字节相等 | `true`、`true` |
| 首个差异／停止原因／数值警告 | `null`／`null`／`{}` |
| 实际及解压基准JSONL SHA256 | `880581b82882313c4abc4c9c838402be6ed04eac18dd5c30ee3780258a27fafa` |
| 相对Reset最终增高 | 33.36294243170235mm，参考z=0.97m |
| 相对沉降最终抬升 | 103.95223232528839mm，零起算行249参考z=0.8994107101064139m |
| 原生完整任务结果 | `native_task_result=false` |
| 严格抓取成功声明 | `strict_grasp_success_claim=false`；未运行额外严格谓词 |

[bounded_grasp.jsonl.gz](bounded_grasp.jsonl.gz)为本次真实轨迹压缩副本，与D1-B基准内容一致。[artifacts.json](artifacts.json)记录来源与SHA及原main/README保护结果。

遗留风险：法兰来源未发现独立许可，维持[原披露](../../../../licenses/SOURCES.md)，本轮不整改；本测试仍只覆盖固定seed白碗抓持诊断，没有完整柜子放置任务、多seed或实机验证。PR保持Draft，未合并main。
