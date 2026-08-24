# 零位移 G-code 运动行处理方案

## 目标分析

真实 SD 文件 `/longer_studio11.gc.gz` 已暴露以下输入：

```text
行号：14
payload：G1X33.696
```

在此前的 `G0` 已将计划机床坐标推进到 `(33.696, 60.404)` 后，该行在绝对模式下的结果为：

```text
start  = (33.696, 60.404)
target = (33.696, 60.404)
```

它包含有效的 X 坐标词，但计算后的 XY 位移为零。当前 `execute_line()` 仅根据“是否出现轴词”决定
是否提交 `MotionRequest`，因此将该行送入 Planner，形成 `steps=0`、`interval=0` 的非法 block，最终
返回 `error:motion-rejected`。

本方案的目标是在 command 层识别这一语义，允许零位移的有效 G-code 行提交其模态状态，但不创建
Planner block、STEP 脉冲或激光运动段。

## 行为定义

### 应接受的零位移行

满足以下条件时，行是零位移运动行：

1. 存在至少一个 X 或 Y 轴词；
2. 坐标、单位、距离模式、位置可信度和所有数值校验均已通过；
3. 使用现有 `MotionExecutionPlan` 换算后，X/Y 的计划步数均为零；
4. Z 轴仍按当前规则拒绝；
5. `G1`、`G0`、`F`、`S`、`M3/M4/M5` 等同一行模态状态仍遵循现有事务提交规则。

处理结果：

```text
execute_line()
  -> 不调用 motion_submit()
  -> 不创建 Planner block
  -> 提交本行合法的 ParserState 模态更新
  -> 返回 ok
```

### 不应放宽的情况

- 单独 `G1` 且没有 X/Y 轴词，继续返回 `error:no-axis-words`。
- 坐标位置为 `Unknown` 或 `Invalidated` 时，含轴词的行继续返回
  `error:position-unknown`，即使结果恰好为零位移。
- 非法数值、超范围进给、非法 S 值、重复词和不支持命令继续返回原有错误。
- 零位移不产生激光 PWM 更新、STEP 脉冲或 Planner execution id。
- 不在 Planner 中接受 `steps=0` 的空 block；Planner 的输入约束保持不变。

## 实现步骤与验收标准

### 阶段一：在 command 层识别 XY 零位移

实现内容：

1. 在 `execute_line()` 完成目标坐标计算后，调用现有 `build_execution_plan()` 完成校验和步数换算。
2. 将“存在轴词”和“存在实际 XY STEP 脉冲”分为两个独立判断。
3. 只有存在实际位移时才调用 `submit_motion_request()`。
4. 零步行保留 `commit_parser_state()`，提交本行的 G/F/S/M 模态状态，但不以前端毫米目标覆盖现有
   的步数网格计划位置。
5. 保留位置可信度检查，并在其后才允许零位移分支返回成功。

验收标准：

- 已知位置 `(33.696, 60.404)` 下发送 `G1 X33.696` 返回一次 `ok`。
- 不出现 `PLANNER-REJECT reason=invalid-motion-block`。
- Planner 已完成 block 数和 Stepper 步数在该行前后不变化。
- `G1` 无轴词仍返回 `error:no-axis-words`。
- 位置未知时发送同一行仍返回 `error:position-unknown`。

结束改动标准：

- command 层不会再为零位移 XY 行创建空 Planner block。
- Planner 无需增加“接受零步 block”的特例。

阶段状态：已完成并实机验证。步数网格量化后的 `(33.7000,60.4000) -> (33.6960,60.4000)`
同样被识别为零 STEP 行；原第 14 行不再产生 `invalid-motion-block`。判定依据已固定为
`MotionExecutionPlan.steps`，而不是浮点毫米坐标的直接相等比较。

### 阶段二：验证模态提交与真实 SD 文件续跑

实现内容：

1. 验证零位移行中携带的 F、S、运动模式等状态仍在后续实际运动中生效。
2. 从已知位置启动 `/longer_studio11.gc.gz`，确认第 14 行不再导致作业失败。
3. 不对每条零位移行增加普通日志，避免灰度文件出现日志压力。
4. 由 SD 文件继续暴露下一条未实现或不兼容命令，再回到增量对齐流程。

验收标准：

- 文件越过第 14 行，`SD-GCODE-REJECT` 不再指向 `G1X33.696`。
- `ok/error` 数量保持每输入行一次最终响应的契约。
- 下一个失败行保留完整的行号、错误和 payload 日志。
- 零位移行前后的实际位移 block 顺序、步数和激光安全状态不回归。

结束改动标准：

- 真实文件已跨越当前零位移 G1 证据行。
- 新发现的问题以新的真实日志为准，不预先扩展无关 G-code 命令。

阶段状态：待开始。真实文件跨过第 14 行后暴露出持续运动供给抖动问题，需先独立完成
解压、SD、Planner 和 RMT 的供给链路排查，再继续将该文件作为后续命令证据来源。

## 关联文件

- `command/command_executor.cpp`：目标计算、位置可信度检查、MotionRequest 提交和 ParserState 事务。
- `motion/motion_submit.cpp`：仅接收存在实际运动的请求，不承担零位移兼容。
- `planner/planner.cpp`：继续拒绝空 block，作为 command 层回归保护。
- `sd/sd_job_service.cpp`：记录真实文件的下一条拒绝证据。
- `command/incremental-gcode-alignment-method.md`：维护真实文件驱动的命令对齐队列。
