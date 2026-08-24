# 机床坐标与步数位置同步方案

## 目标分析

当前 `ray2-idf` 使用 `ParserState.position_mm` 推断后续 G-code 的起点。该值在运动请求成功进入 Planner 后立即更新，因此它表示“已规划的目标位置”，不表示电机已经执行到的位置。

RayCube 使用开环步进电机，没有编码器，固件无法直接测量电机的物理位置。可建立的坐标依据是：以一个已知位置为起点，按照实际输出的 STEP 脉冲数量和方向累计推导机床位置。

本方案参考 `ray2-main` 的以下结构：

```text
sys_position[]longer_studio11.gc.gz
    -> 步进执行层累计的机床步数

system_convert_array_steps_to_mpos()
    -> 步数转换为机床毫米坐标

gc_state.position
    -> G-code 解释器使用的机床坐标

coord_system + coord_offset
    -> G54/G92 工作坐标偏移
```

本方案的目标不是制造不存在的传感器反馈，而是建立可追踪的开环位置模型，并明确哪些情况下该模型失效。

## 当前问题

当前状态链路如下：

```text
G-code
    -> ParserState.position_mm
    -> MotionRequest
    -> Planner block
    -> STEP/DIR
```

存在以下缺口：

1. 没有独立的机床步数位置变量。
2. STEP 脉冲执行后没有更新位置计数。
3. Planner 接收位置和 Stepper 执行位置没有明确同步点。
4. `planner_reset()` 不会使解析器坐标失效。
5. SD 新任务开始时不会根据已知起点重建坐标状态。
6. 用户手动移动电机后，系统无法知道位置已经改变。
7. G54 当前只有零偏移状态，尚未参与完整的工作坐标到机床坐标转换。

因此，当前 `ParserState.position_mm` 应被视为 `planned_position_mm`，不能直接称为实际坐标。

## 坐标定义

### 1. 机床坐标 MPos

机床坐标以 RayCube 的已知基准点为零点，单位为毫米：

```text
X: 0 ~ 130 mm
Y: 0 ~ 110 mm
```

它由机床步数换算得到：

```text
machine_position_mm[axis] = machine_position_steps[axis] / steps_per_mm[axis]
```

方向由 STEP/DIR 的正负方向决定。

### 2. 工作坐标 WPos

G-code 中的绝对坐标属于工作坐标。第一版只支持 G54：

```text
machine_target = work_target + g54_offset
```

当前 G54 偏移为零，因此工作坐标与机床坐标数值相同；但实现必须保留转换边界。

### 3. 计划位置

Planner 接受 block 后，解析器必须知道后续绝对坐标的起点，因此需要保存：

```text
planned_machine_position
```

它表示所有已经成功进入 Planner 的 block 的末端位置，不代表这些 block 已完成执行。

### 4. 执行位置

Stepper 每实际输出一个 STEP 脉冲，就更新：

```text
executed_machine_position_steps
```

再转换得到：

```text
executed_machine_position_mm
```

该值是开环推算的执行位置。没有编码器时，不能保证它与被手动移动后的物理位置一致。

## 与 RAY2-main 的对应关系

| RAY2-main | ray2-idf 目标结构 | 责任 |
| --- | --- | --- |
| `sys_position[]` | `machine_position_steps[2]` | Stepper 执行层累计步数 |
| `system_convert_array_steps_to_mpos()` | `machine_position_mm()` | 步数转换为毫米 |
| `gc_state.position` | `planned_machine_position_mm` | 解释器连续计算绝对目标 |
| `gc_state.coord_system` | `g54_offset_mm[3]` | 工作坐标系偏移 |
| `gc_state.coord_offset` | 后续 `g92_offset_mm[3]` | 暂不实现，预留 |
| `gc_sync_position()` | `position_sync_from_machine()` | 启动、复位、回零后同步 |
| Stepper ISR 更新 `sys_position` | Planner/Stepper 更新步数 | 记录实际输出的 STEP |
| `get_wco()` | `work_coordinate_offset()` | 计算工作坐标偏移 |

RAY2-main 的 `sys_position` 也是开环计数，不是编码器反馈。它的可靠性来自：步数由 Stepper 执行层维护，并在启动、回零、复位和异常路径中重新同步。

## 实现原则

### 1. Planner 仍然使用计划位置

连续 G-code 不能等待前一个 block 完全执行后再解析，否则会直接降低串口流和 SD 流吞吐。因此：

```text
解析器使用 planned_machine_position 计算下一个目标
Planner 保存 block 的 start/target
Stepper 异步执行 block
```

### 2. 执行位置由 STEP 脉冲驱动

位置计数只能在确定 STEP 脉冲已经发出时更新，不能在 G-code 解析成功时更新执行位置。

### 3. 位置状态必须有可信度

增加明确的位置状态：

```text
Unknown
Known
Invalidated
```

以下情况必须将位置标记为 `Unknown` 或 `Invalidated`：

- 上电但未完成回零或基准点确认；
- 用户手动移动电机；
- 电机失步可能发生；
- 急停、复位或 Planner 丢弃当前 block；
- Stepper/RMT 执行失败；
- 作业中途更改物理坐标基准。

### 4. 坐标同步必须是显式操作

系统不能假设 `planner_reset()` 自动知道电机在哪里。同步接口必须明确表达前提：

```text
position_sync_from_known_origin()
position_invalidate()
position_set_machine_mm()
```

其中 `position_set_machine_mm()` 只能在用户确认电机已处于该位置时使用；它不是物理校准，也不替代回零。

## 实现步骤与验收标准

### 阶段一：建立机床步数位置所有权

实现内容：

1. 在 `machine` 或 `stepper` 位置所有者中增加 X/Y 有符号步数计数。
2. 明确只有 Stepper/RMT 执行路径可以修改该计数。
3. 明确正负方向与 `positive_direction` 的对应关系。
4. 提供只读的机床步数和机床毫米位置查询接口。
5. 增加位置可信度状态。

验收标准：

- 单轴正向运动后，步数增加与执行 STEP 数量一致。
- 单轴反向运动后，步数减少与执行 STEP 数量一致。
- XY 直线运动中，两个轴的累计步数分别正确。
- Planner 仅入队但尚未执行时，执行位置不提前变化。
- 位置查询不会直接读取或修改 `ParserState.position_mm`。

结束改动标准：

- 已明确机床步数的唯一写入者。
- 已能区分计划位置和执行位置。
- 实机短线运动能验证步数方向和数量。

阶段状态：已完成并实机验证。连续执行 X 正向 1 mm、X 反向 1 mm、XY 正向 1 mm、XY 反向
1 mm 后，Stepper 日志依次记录 `(80,0)`、`(0,0)`、`(80,80)`、`(0,0)` 步；位置可信度在未
完成原点确认前保持 `Unknown`。

### 阶段二：建立 Planner block 的位置边界

实现内容：

1. 在 Planner block 中保留机床坐标起点和目标。
2. block 入队时只更新计划位置。
3. block 完成时发布执行完成事件。
4. 执行完成事件中携带 execution position 或对应步数。
5. block 失败、取消和 reset 时不把目标误记为已执行。

验收标准：

- 连续多个 block 的计划起点按入队顺序衔接。
- 执行位置只在对应 block 完成后变化。
- Feed Hold、Reset 和 BlockFailed 后，状态不会伪造为目标已完成。
- Planner 队列满时，计划位置和执行位置都不会错误前进。

结束改动标准：

- 能分别打印或查询 planned position 与 executed position。
- 一次故意触发的 Planner 拒绝不会污染后续绝对坐标。

阶段状态：已完成并实机验证。每个完成 block 的日志同时输出 Planner 冻结的机床起点、目标和
Stepper 执行步数快照；连续正反向与 XY 短线的 planned 边界连续，执行步数回到预期 `(0,0)`。

### 阶段三：实现解析器与机床坐标同步

实现内容：

1. 将当前 `ParserState.position_mm` 重命名或明确为计划机床位置。
2. 增加从已知机床位置重置解析器位置的接口。
3. SD 作业启动前要求位置状态为 `Known`，或由明确的“基准点确认”步骤建立位置。
4. SD 作业失败、复位和急停后使位置失效。
5. 禁止在位置未知时继续执行依赖绝对坐标的作业。

验收标准：

- 电机处于已知 `(0,0)` 后建立基准，`G1 X33.696` 能实际移动到约 `33.696 mm`。
- 作业过程中手动改变电机位置后，系统不会继续假装坐标可信。
- 发生 reset 后，下一条绝对坐标命令不会沿用旧作业位置。
- 无效位置状态能够通过状态接口或日志观察到。

结束改动标准：

- 已解决“物理回到零点但 ParserState 仍为旧坐标”的问题。
- 未引入静默自动归零或未经确认的坐标重置。

### 阶段四：接入 G54 工作坐标转换

实现内容：

1. 保留工作坐标和机床坐标的独立表示。
2. G54 选择当前工作坐标系。
3. 默认 G54 偏移为零。
4. 绝对模式下执行：`machine_target = work_target + g54_offset`。
5. 坐标转换后再执行 `[0,130] x [0,110]` 幅面检查。
6. 暂不实现 G10、G92 和 G55-G59。

验收标准：

- G54 零偏移时，工作坐标与机床坐标行为保持现有结果。
- 非零测试偏移下，目标转换方向和数值正确。
- 转换后的机床目标越界时被拒绝。
- G54 模态行本身不产生运动。

结束改动标准：

- G-code 坐标、G54 偏移、机床幅面三者边界清晰。
- 坐标转换不会绕过现有软限位。

### 阶段五：异常与恢复路径验收

实现内容：

1. 为复位、急停、Planner reset、Stepper 执行失败增加位置失效处理。
2. 为已知原点同步提供唯一入口。
3. 为 SD 作业开始、结束和失败记录位置状态。
4. 防止 UART 与 SD 输入路径分别重置或修改坐标状态。

验收标准：

- 任意异常结束后查询到的位置状态为未知或失效，而不是旧的 Known。
- 重新确认基准点后才能恢复绝对坐标作业。
- SD 作业结束后，计划位置和执行位置一致或差异可诊断。
- 位置日志不会逐行打印，正常灰度流无额外日志压力。

结束改动标准：

- 坐标生命周期覆盖启动、运行、完成、失败、复位和恢复。
- 实机短线、连续短线和一次异常中断均通过验证。

## 明确不实现的内容

- 不把开环步数计数描述为编码器真实反馈。
- 不在本方案中实现硬件回零开关驱动。
- 不自动推断用户手动移动后的物理位置。
- 不为了让 SD 文件继续运行而静默清零坐标。
- 不在本方案中实现完整 G10、G92、G55-G59。
- 不让 protocol 直接操作 STEP 或修改执行位置。

## 最终判断标准

本方案完成后，系统必须能够明确回答：

```text
当前工作坐标是什么？
当前计划机床坐标是什么？
Stepper 已经输出了多少步？
根据步数推算的执行机床坐标是什么？
这个坐标依据是否仍然可信？
```

如果其中任一项无法回答，系统必须报告位置未知或位置失效，而不是继续使用旧坐标执行绝对运动。
