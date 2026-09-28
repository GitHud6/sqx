# SD 文件与作业模块

`sd` 负责文件枚举、普通 G-code 读取和将完整 G-code 行提交至协议执行路径。

## 当前行为

- 支持 SD 文件列表和未压缩 G-code 作业；`.gz` 文件可保存但不能作为作业执行。
- 文件读取器只返回完整逻辑行，G-code 语义仍由 `command` 解析。
- 作业具有 starting、running、draining、completed、faulted 生命周期。
- 作业期间的解析失败、执行失败或位置失效会停止供料并保留 fault 原因。

作业结束后 planner、segment 与实际位置都会回到空闲状态。
