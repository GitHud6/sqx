# G-code 解析与运动请求接口设计

## 目标

本模块的目标不是立即复制 RAY2 的完整 `GCode.cpp`，而是先建立稳定的边界：

```text
完整命令行
  -> execute_line()
  -> 解析并校验
  -> MotionRequest
  -> motion_submit()
  -> Planner
```

当前项目已经完成 UART 行帧化和最小 `execute_line()` 验证。下一步必须把“文本解析”和“运动执行”分开，使后续移植 Planner、步进时序和激光功率控制时，不需要再次改动协议层。

## 关键原则

### 1. 一行命令是一个事务

`execute_line()` 必须先把当前 `ParserState` 复制为临时解析上下文，再解析整行并完成校验。只有运动请求成功提交给 `motion` 后，才允许更新正式的解析状态。

```text
ParserState
    -> ParserContext 临时副本
    -> 解析 G/X/Y/F/S/M
    -> 校验
    -> 生成 MotionRequest
    -> motion_submit() 成功
    -> commit ParserState
```

解析失败时，正式状态不能发生任何变化。Planner 满、软限位、急停或其他运动层拒绝请求时，也不能提前更新解析器位置。

这对应 RAY2 中 `gc_state`、`gc_block`、`plan_line_data_t` 和 `mc_line()` 的提交关系，是移植时最重要的语义。

### 2. 解析层不直接访问硬件

`command` 层不得调用 GPIO、`stepper_pulse()`、UART 驱动或激光 PWM。它只负责：

- 识别 G-code 词
- 检查数字和模态合法性
- 将英寸转换为毫米
- 根据 G90/G91 计算绝对目标坐标
- 生成结构化运动请求
- 在运动层成功后提交模态状态

`protocol` 层只负责接收完整 payload、调用 `execute_line()` 和返回 `ok/error`。

### 3. S 值必须和运动请求绑定

灰度雕刻的典型输入是：

```text
G1 X0.05S111
G1 X0.05S103.9
```

每一行的 `S` 都是该段运动的激光功率目标，不能只更新一个全局 `spindle_speed`。因此 `MotionRequest` 必须携带该行最终生效的 `spindle_power`，使 Planner/Stepper 或后续激光执行器能够按 block 使用它。

## 目标数据结构

实际实现时可先放在 `command` 的公共头文件中，待 `motion` 模块建立后再移动到共享接口文件。

### ParserState

`ParserState` 表示已经被成功接受的解释器状态：

```text
system_state
distance_mode       G90 / G91
units               G20 / G21
motion_mode         G0 / G1
position_mm[3]      当前解释器认为的绝对目标位置
feed_rate_mm_per_min 当前有效进给
spindle_speed       当前有效激光功率/主轴值
```

它不是电机实时位置。真实位置属于未来的 `system`/`stepper` 状态，不能用解析器位置替代。

### ParsedBlock

`ParsedBlock` 是单行的临时解析结果，不应跨命令保存：

```text
has_motion / motion_mode
has_units / units
has_distance_mode / distance_mode
has_axis[3] / axis_value[3]
has_feed_rate / feed_rate
has_spindle_speed / spindle_speed
has_spindle_command / M3/M4/M5
```

它保存“本行写了什么”，不保存“最终状态是什么”。最终状态由 `ParserContext` 与当前 block 合并得到。

### MotionRequest

`MotionRequest` 是 command 到 motion 的稳定契约，建议包含：

```text
motion_mode             Rapid 或 Linear
target_mm[3]             已转换为绝对毫米坐标的目标
start_mm[3]              提交时的解释器起点，用于校验和调试
feed_rate_mm_per_min     已转换的进给
spindle_power            本行生效的 S 值
spindle_mode             Off / CW / CCW 或 Laser mode
has_motion               是否需要产生运动 block
```

第一版只支持 RayCube 的 X/Y 两个实际轴；Z 可以继续保留为 0，但不能让未知轴悄悄产生运动。

## `execute_line()` 的目标流程

### 第一步：预处理

1. 接受不包含 `\r`、`\n` 的完整 payload。
2. 空行直接返回 `Ok`，不改变状态。
3. 处理 `;` 和括号注释。
4. 支持字母与数字之间无空格，例如 `X0.05S111`。
5. 统一字母大小写。

### 第二步：词法解析

逐个读取 `letter + number`：

- `G`：暂时支持 `G0/G1/G20/G21/G90/G91`
- `M`：建议增加 `M3/M4/M5`
- `X/Y`：RayCube 实际运动轴
- `Z`：第一版可识别但拒绝实际运动，或明确返回 unsupported
- `F`：进给，必须非负且有限
- `S`：激光功率/主轴值，必须非负且有限

每个词进入 `ParsedBlock` 前检查重复。一个 modal group 同一行出现两个命令时返回 `ModalGroupViolation`，例如 `G90 G91`。

### 第三步：合并模态

以当前 `ParserState` 为基线复制出临时上下文：

```text
next = state
next.units = 本行 G20/G21，否则 state.units
next.distance_mode = 本行 G90/G91，否则 state.distance_mode
next.motion_mode = 本行 G0/G1，否则 state.motion_mode
next.feed_rate = 本行 F，否则 state.feed_rate
next.spindle_speed = 本行 S，否则 state.spindle_speed
```

单位转换必须在生成目标前完成：

```text
G20: value_mm = value_inch * 25.4
G21: value_mm = value
```

### 第四步：计算目标

对每个已出现的轴词：

```text
G90: target_mm[axis] = value_mm
G91: target_mm[axis] = state.position_mm[axis] + value_mm
```

未出现的轴保持原位置。仅有 `F` 或 `S` 的行可以是状态更新，不应生成空运动 block。`G1` 没有轴词时，应按目标行为决定是仅切换模态还是返回 `NoAxisWords`；第一版建议遵循 RAY2，对没有实际轴运动的模态行不提交 Planner。

### 第五步：运动层提交

如果存在轴运动，构造 `MotionRequest` 并同步调用：

```cpp
MotionSubmitResult result = motion_submit(request);
```

`motion_submit()` 的职责是检查 Planner 空间、软限位、进给范围，并在允许时将请求放入 Planner。它可以阻塞等待 Planner 空间，但必须持续检查 reset/feed hold 等实时状态。

### 第六步：提交状态

只有以下条件同时成立时，才执行：

```text
state = next;
```

- 解析成功
- 参数和模态校验成功
- 没有软限位错误
- `motion_submit()` 接受请求，或状态命令明确执行成功

返回 `Ok` 后，`protocol` 才向 UART 输出 `ok\r\n`。

## 与当前模块的关系

```text
protocol/line_assembler
    只负责 CR/LF 行帧化

protocol/command_dispatch
    只负责调用 execute_line 与报告结果

command/command_executor
    负责 G-code 解析、校验、状态事务和 MotionRequest 生成

motion
    负责 MotionRequest -> Planner 的提交

planner
    负责 block 缓冲、前瞻、速度和加速度规划

stepper
    负责执行已规划的 STEP/DIR 时序
```

当前 `command_dispatch()` 中的静态 `ParserState` 可以继续保留，但最终应由 command/runtime owner 统一持有，避免将来多个命令来源共享状态时产生并发写入。

## 第一批实现范围

先完成以下闭环，不引入弧线、坐标系、探测、刀补、暂停、SD 和完整系统命令：

```text
G90/G91/G20/G21
G0/G1
X/Y/F/S
M3/M4/M5
注释
重复词和模态冲突
负值、NaN、Inf、范围错误
```

LightBurn 灰度行应能够被解析为独立的 `MotionRequest`，例如：

```text
G1 X0.05S111
-> target_mm.x = previous_x + 0.05（G91 时）
-> feed_rate = 当前有效 F
-> spindle_power = 111
-> motion_mode = Linear
```

在真实 Planner 尚未完成前，`motion_submit()` 可以先实现为记录请求并返回成功，但必须保持相同接口，不允许 `execute_line()` 直接修改位置来伪装运动已经完成。

## 验收标准

### 解析层

- `G1 X0.05S111` 能产生包含 X 位移和 S 功率的结构化请求。
- 连续灰度行中每行的 S 值不串行、不丢失。
- G90/G91、G20/G21 的目标计算正确。
- 非法行只返回一次 `error`，且不会污染下一行状态。
- 解析失败时 `ParserState` 完全不变。

### 运动层桩接口

- 能记录每次 `MotionRequest` 的起点、目标、进给和功率。
- 不直接操作 GPIO。
- Planner 未准备好时，可以明确返回 `Busy`，不能返回成功后丢弃请求。

### 进入真实 Planner 前的停止条件

只有以下条件全部满足，才开始实现 Planner：

- `execute_line()` 不再直接调用 `stepper`。
- ParserState 只在 motion 提交成功后更新。
- `MotionRequest` 能完整携带灰度行的 S 值。
- 连续短行压力测试没有出现状态错位、重复响应或请求丢失。
- 失败请求后的下一条合法 G-code 可以正常执行。

## 实施阶段与阶段验收

### 阶段一：重构 `execute_line()` 的输出边界

阶段状态：代码已完成，待构建和实机协议验收。

实现内容：

1. 保留当前 `ParserState` 作为已提交状态。
2. 新增 `ParsedBlock` 和 `MotionRequest` 类型。
3. 将当前直接修改 `position_mm` 的逻辑改为先生成临时目标。
4. 在 motion 接口未接入前，使用明确的 `motion_submit()` 桩函数承接请求。

阶段验收：

- `execute_line()` 不再直接调用 `stepper` 或 GPIO。
- `MotionRequest` 能携带起点、目标、进给和 S 值。
- 仅解析状态命令时可以提交状态，不生成空运动请求。
- 代码能够区分“解析成功”和“运动提交成功”。

### 阶段二：扩展 LightBurn 所需 G-code 语法

阶段状态：代码已完成，待构建和连续灰度输入验收。

实现内容：

1. 支持 `G0/G1/G20/G21/G90/G91`。
2. 支持 `M3/M4/M5`，并把激光模式放入请求。
3. 支持无空格连续词，例如 `G1 X0.05S111`。
4. 支持分号注释和括号注释。
5. 增加负数、重复词、模态组冲突、非有限数和范围检查。

阶段验收：

- `G1 X0.05S111` 能得到 X 位移和功率 111。
- 连续灰度行的每个 S 值与输入顺序一致。
- G90/G91 和 G20/G21 的目标坐标计算正确。
- 非法命令只返回一次错误，且不污染下一条命令。
- 解析器能够继续处理 LightBurn 的短行高频输入。

### 阶段三：实现 motion 桩接口

阶段状态：代码已完成，待构建和协议实机验收。

实现内容：

1. 建立 `motion` 文件夹及 `motion_submit()` 接口。
2. 桩接口只记录或输出 `MotionRequest`，不操作 GPIO。
3. 定义 `MotionSubmitResult`，至少区分 `Accepted`、`Busy`、`Rejected`。
4. 预留 Planner 满、软限位、急停和非法进给的错误映射。

阶段验收：

- 每个有效运动行都生成一个且仅一个 `MotionRequest`。
- 请求中的起点、目标、F、S、运动模式可观测。
- `motion` 不依赖 UART 和 stepper。
- 返回 `Busy` 或 `Rejected` 时，调用方不会提交解析状态。

### 阶段四：实现解析状态事务提交

阶段状态：代码已完成，待构建和协议实机验收。

实现内容：

1. 解析开始时复制 `ParserState` 到临时上下文。
2. 在临时上下文中应用本行模态、单位和目标计算。
3. 先调用 `motion_submit()`。
4. 只有返回 `Accepted` 后才将临时上下文写回 `ParserState`。
5. 状态命令和运动命令分别定义提交规则。

阶段验收：

- 语法错误后 `ParserState` 完全不变。
- motion 返回 `Busy` 或 `Rejected` 后位置、F、S、模态均不提前改变。
- 接受运动后下一条 G91 命令以新目标为起点。
- `ok` 只在解析和运动提交都成功后发送。

### 阶段五：连续灰度压力验收

实现内容：

1. 使用 508 DPI、6000 mm/min 的 LightBurn 风格短行作为输入。
2. 连续发送包含 `X0.05`、`X0.1` 和变化 S 值的行。
3. 统计 UART 队列、解析响应和 MotionRequest 数量。
4. 记录 motion 桩耗时，为后续 Planner 背压设计提供基线。

阶段验收：

- 输入行数、响应数和请求数一致。
- 每个请求的 S 值没有错位、覆盖或丢失。
- UART 接收队列没有持续增长或溢出。
- 非法行不会破坏后续合法灰度行。
- 完成后才进入真实 Planner，不在压力测试阶段接入步进 GPIO。

## 明确不做

本设计阶段不移植以下 RAY2 依赖：

- 完整 `$` 系统命令和 NVS 设置
- G2/G3 弧线
- G10/G28/G30/G92 坐标系命令
- 探测、回零、软限位完整逻辑
- M7/M8/M9、刀具和冷却液
- WiFi、WebUI、SD、屏幕和 APP 兼容层

这些功能应在基础 G-code 到 Planner 闭环稳定后按独立模块增加。
