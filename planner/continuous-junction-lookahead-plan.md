# 连续 Junction 前瞻与灰度雕刻运动规划

## 目标

让当前运动规划从“每个 block 独立完成一次起步、加速、减速、停止”，升级为
“多个连续 block 共享速度规划，在 junction 处保持合理的入口和出口速度”。

最终目标是满足 Ray2/LightBurn 灰度雕刻的连续运动需求：大量短距离、连续方向变化的
`G1` 指令不能因为 block 边界而反复降速到零，也不能因为 planner 前瞻冻结过早而退化为
局部短视规划。

## 当前结论

当前工程并非没有加减速或 junction 公式：

- `stepper_prep` 已经能为单个 block 生成梯形或三角形速度曲线；
- `planner` 已经计算 `entry_speed_sqr`、`exit_speed_sqr` 和基础 junction 速度；
- RMT、GPTimer、segment ring 和实际 STEP 输出已经打通。

当前真正的问题是前瞻结果没有持续作用于实际执行链路。已有 segment 后，planner 会因为
`s_segment_prepared_blocks != 0` 提前停止重算，导致后续尚未生成 segment 的 block 无法
继续参与前瞻。灰度短线段因此容易变成：

```text
block 1: 0 -> 加速 -> 0
block 2: 0 -> 加速 -> 0
block 3: 0 -> 加速 -> 0
```

而目标行为应接近：

```text
block 1 -> block 2 -> block 3
       通过 junction 连续运行
```

## 设计边界

### 可以重新规划的内容

- 尚未转换为 `StepSegment` 的 block；
- 尚未交给执行器的 block；
- 新提交 block 与未冻结 block 之间的 junction 参数；
- 未冻结 block 的入口速度、出口速度和梯形/三角形 profile。

### 不可重新规划的内容

- 正在 GPTimer ISR 中执行的 segment；
- 已经发布到 segment ring 的 segment；
- 已经由 RMT 后端开始执行的 STEP 时序；
- 已经完成的 block。

规划冻结边界必须由明确的 block/segment 所有权表示，不能用“已经有任意 segment
生成”作为整个 planner 的冻结条件。

## 目标链路

```text
UART / SD / 其他供料源
        |
        v
execute_line() -> planner block ring
                         |
                         v
              未冻结 block 的前瞻规划
              - junction 速度
              - entry speed
              - exit speed
              - 加减速可达性
                         |
                         v
                 stepper_prep
                         |
                         v
                 SPSC segment ring
                         |
                         v
                   GPTimer ISR
                         |
                         v
                    RMT STEP
```

`stepper_prep` 只消费已经冻结的 block profile，不负责重新决定 block 之间的
junction 速度。GPTimer ISR 和 RMT 不参与 planner 语义计算。

## 实现步骤

### 阶段一：建立可观测的 block 速度状态

实现内容：

1. 为 `$POS/STATUS` 或 planner 状态增加必要的诊断字段，至少可以观察当前 block 和
   后续 block 的 `entry_speed`、`exit_speed`、`max_junction_speed`。
2. 区分以下状态：
   - block 已入队但未规划；
   - block 已完成前瞻但未生成 segment；
   - block 已生成 segment 并冻结；
   - block 正在执行；
   - block 已完成。
3. 使用连续同方向短线段验证当前实现是否出现大量 `entry=0/exit=0`。

验收标准：

- 连续提交至少 8 个同方向短 `G1` 后，可以观察每个 block 的入口/出口速度；
- 能明确区分“未冻结 block”和“已冻结 block”；
- 当前问题可以通过诊断数据复现，而不是只依赖电机声音判断；
- 不修改 RMT、GPIO、GPTimer 和已验证的 STEP 输出逻辑。

当前实现状态：已增加 `PlannerDiagnostics` 只读快照，并由 `$POS/STATUS` 输出
`planner=(queued,frozen,completed)` 与前三个 block 的
`lookahead=(state,id,entry,exit,junction,nominal)`。

实机验收记录：在 `$POS/SET=0,0`、`G91` 后，一次性发送首段
`G1 X20 F600` 与 7 条 `G1 X0.05 F600`。首段执行中观测到：

```text
planner=(queued=8,frozen=1,completed=32)
lookahead=(
  b0=executing#33:e=0.0,x=0.0,j=0.0,n=600.0;
  b1=lookahead#34:e=0.0,x=0.0,j=0.0,n=600.0;
  b2=lookahead#35:e=0.0,x=0.0,j=0.0,n=600.0
)
```

该结果证明：后续同方向 block 已入队，但当前执行 block 已被冻结为 `exit=0`，未冻结
短 block 也未获得 junction、入口或出口速度。阶段一验收通过；进入阶段二前需提交本阶段改动。

### 阶段二：修正前瞻冻结边界

实现内容：

1. 移除“只要已有 segment 就停止 planner 重算”的全局限制。
2. 增加明确的 planner planned/frozen 边界，至少能够表示：
   - 已冻结 block 的起点；
   - 尚未冻结 block 的起点；
   - 当前执行 block；
   - 新提交 block 所在位置。
3. 新 block 到达时，只重算尚未冻结区域及其与冻结区域的连接 junction。
4. block 被完成并回收后，正确推进前瞻边界，避免使用已经回收的 block 数据。

验收标准：

- segment ring 已经开始消费时，新提交的 block 仍能参与后续前瞻；
- 已经执行的 segment 不会被修改或重新发布；
- 已冻结 block 的速度 profile 保持不变；
- 连续提交短线段不会因为 planner 重算引入数据竞争、重复 block 或位置错误；
- 单轴、XY 直线、正反方向切换仍能通过现有实机测试。

当前实现状态：planner 已将已发布 segment 的 source block 前缀视为冻结窗口，仅跳过该
前缀的重算；冻结前缀之后的 block 会继续执行 junction、反向减速和正向加速 pass。首个
未冻结 block 从冻结前缀既定的出口速度衔接，绝不反向修改已发布 segment。

实机验收记录：沿用阶段一的首段 `G1 X20 F600` 加 7 条同方向 `G1 X0.05 F600`，首段
执行期间得到：

```text
planner=(queued=8,frozen=1,completed=0)
lookahead=(
  b0=executing#1:e=0.0,x=0.0,j=0.0,n=600.0;
  b1=lookahead#2:e=0.0,x=424.3,j=inf,n=600.0;
  b2=lookahead#3:e=424.3,x=600.0,j=inf,n=600.0
)
```

`b0` 已在后续 block 到达前冻结，故其出口仍为零；`b1` 和 `b2` 已获得连续的
`exit/entry=424.3 mm/min`。共线同向 junction 的 `j=inf` 符合几何约束。阶段二验收通过；
进入阶段三前需提交本阶段改动。

### 阶段三：实现连续 junction 速度

实现内容：

1. 对相邻可连续运动的 block 计算 junction 最大允许速度。
2. 使用反向 pass 计算减速可达性。
3. 使用正向 pass 计算加速可达性。
4. 确保前一 block 的出口速度与后一 block 的入口速度一致。
5. 对以下情况强制恢复到安全低速或零速：
   - 运动类型不连续；
   - 方向反向；
   - 激光控制语义要求停机；
   - feed、单位或系统状态不允许连续运行；
   - block 无法满足幅面或速度约束。
6. 保持当前 junction deviation 和 Ray2 目标参数可配置，暂不通过修改参数掩盖规划错误。

验收标准：

- 同方向连续 block 的中间 junction 速度不再全部为零；
- 直线连续段在 block 边界没有人为停顿；
- 90 度转角会按 junction 约束降速；
- 反向运动会安全减速，不产生错误的高速穿越；
- 起点和终点仍保持正确的零速度约束；
- planner 记录的最终位置与实际执行步数一致。

### 阶段四：segment 级速度连续性

实现内容：

1. `stepper_prep` 使用 planner 已确定的 block 入口/出口速度生成 segment。
2. 相邻 block 的首尾 segment 速度保持连续，避免 segment 边界再次强制归零。
3. segment ring 低水位补充不能改变已经冻结的速度 profile。
4. GPTimer ISR 继续只负责消费 segment 和产生时序，不读取 planner 或执行复杂速度计算。

验收标准：

- segment 边界没有软件引入的速度断层；
- segment ring 在连续运动期间不出现 underflow；
- GPTimer ISR 不调用 planner、文件系统、日志或阻塞 API；
- 运动连续期间轴 enable 不会在相邻 block 之间反复切换；
- 单段运动和多段连续运动的 STEP 数量均正确。

### 阶段五：LightBurn 灰度雕刻验收

实现内容：

1. 使用与 Ray2-main 一致的灰度文件、速度和 DPI 参数：
   - `F6000`；
   - `508 DPI`；
   - 每像素约 `0.05 mm` 的短线段；
   - 连续 `S` 功率变化。
2. 观察 planner block 队列、冻结边界、entry/exit speed、segment 水位和 underflow。
3. 验证激光功率更新发生在对应 segment 实际开始执行时。
4. 验证文件结束、`M5/M9`、异常和 reset 后轴与激光都进入安全状态。

验收标准：

- 灰度雕刻可以完整执行到文件结束；
- 连续短线段不再出现明显的 block 边界停顿；
- 电机不出现由每个 block 反复起停造成的明显“嘟嘟嘟”节奏；
- 雕刻图案不出现由速度断层造成的明显扭曲或局部堆料；
- planner、segment、RMT 均无 underflow、reset 或 watchdog；
- 实际步数、最终坐标和输入文件一致。

## 不纳入本计划

- 新增 G-code 语句；
- `M9` 等厂商私有命令的业务对齐；
- SD、gzip、UART 和其他供料源实现；
- 机械传动、皮带张力、电机电流和微步硬件校准；
- RMT、GPTimer 和 GPIO matrix 的再次重构；
- 通过降低速度或提高脉冲数量掩盖 planner 连续性问题。

## 结束改动标准

满足以下条件后，本计划结束：

1. planner 能持续对所有未冻结 block 做 junction 前瞻；
2. 已冻结 block、正在执行 block 和未冻结 block 的所有权边界清晰；
3. 相邻连续 block 的入口/出口速度能够连续传递；
4. 单轴、XY、转角、反向和停止场景均通过实机验证；
5. LightBurn 灰度文件能够完整执行，且不再因 block 级反复归零产生明显抖动；
6. 每个阶段完成实机验收后提交对应改动，再进入下一阶段；
7. 文档中的所有验收结果已补充实际日志和测试命令。

## 参考实现

- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Planner.cpp`
- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Planner.h`
- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Stepper.cpp`
- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Stepper.h`
- `planner/planner.cpp`
- `stepper/stepper_prep.cpp`
- `stepper/timer_stepper_executor.cpp`
