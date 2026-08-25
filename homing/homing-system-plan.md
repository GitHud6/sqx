# RayCube `$H` 系统归零方案

## 目标分析

RayCube 的 X/Y 限位分别位于机械负方向。`$H` 的目标不是将当前脉冲计数直接清零，
而是通过限位开关建立一次可信的机床坐标基准；只有完整流程成功后，系统才允许把
`PositionState` 变为 `Known`。

参考 `ray2-main` 的 RayCube 配置：

| 参数 | X | Y |
| --- | ---: | ---: |
| 限位 GPIO | GPIO42 | GPIO2 |
| 限位方向 | 负方向 | 负方向 |
| 最大幅面 | 130 mm | 110 mm |
| 步距换算 | 80 steps/mm | 80 steps/mm |

归零速度和回退参数：

```text
seek rate      = 2000 mm/min
locate rate    = 500 mm/min
pull-off       = 3 mm
```

`ray2-main` 使用 `HOMING_POS_WITHOUT_PULLOFF`：机械在最终触发后会物理回退，
但软件机床坐标仍写为 `(0,0)`。因此本项目也将“限位触发处”定义为逻辑零点，
不把回退后的物理位置记为 `(3,3)`。

## 范围与边界

`$H` 是协议系统命令，不进入 `execute_line()`，也不是普通 `G0/G1` 或 Planner block。

```text
UART "$H"
  -> protocol command dispatch
  -> homing service task
  -> limit driver / system-motion executor
  -> stepper machine position
  -> ParserState synchronization
```

职责分配：

- `machine`：定义限位引脚、有效电平、归零方向、速度、回退距离和幅面参数。
- `driver`：拥有 GPIO 输入、去抖、限位状态快照和 ISR 安全停止接口。
- `homing`：拥有归零状态机、超时、轴顺序、失败原因与最终坐标建立。
- `stepper`：执行归零专用 system motion；只在实际 STEP 脉冲后更新脉冲计数。
- `planner`：归零开始前必须 idle；归零期间不接收普通运动；不参与限位判断。
- `protocol`：识别 `$H`、返回一次启动结果、报告状态，不执行归零细节。

安全约束：

1. 归零开始前关闭激光。
2. X/Y 首版顺序归零，不并行归零。
3. 限位 ISR 不执行解析、日志、坐标写入或 Planner 操作；仅停止脉冲并通知归零任务。
4. 任一轴超时、触发后无法退离、二次定位失败或 reset 时，位置保持 `Invalidated`。
5. 只有 X 与 Y 都成功后，才调用 `stepper_set_machine_position_mm({0,0})` 并同步 ParserState。

## 目标状态机

```text
Idle / PositionUnknown / PositionInvalidated
  -> StartRequested
  -> HomingXSeek
  -> HomingXPullOff
  -> HomingXLocate
  -> HomingXFinalPullOff
  -> HomingYSeek
  -> HomingYPullOff
  -> HomingYLocate
  -> HomingYFinalPullOff
  -> CommitOrigin
  -> Idle / PositionKnown

任意阶段失败
  -> Faulted / PositionInvalidated
```

最终回退完成后，物理滑架位于开关外约 `3 mm`，但机床坐标为：

```text
machine position = (0.0000, 0.0000)
planned position = (0.0000, 0.0000)
```

后续正向 `G1 X... Y...` 从幅面左下角逻辑零点向工作幅面内部移动。

## 实现步骤与验收标准

### 阶段一：建立 `$H` 命令与归零状态边界

实现内容：

1. 在 `protocol` 中识别大小写无关的 `$H`，不交给 `execute_line()`。
2. 新建 `homing` 服务接口与只读状态：`idle`、`running`、`completed`、`faulted`。
3. `$H` 仅在 Planner idle、SD 无活动作业、无归零任务时接受。
4. 接受后立即关闭激光并将普通运动入口置为 busy；本阶段不允许伪造位置已知。
5. 在没有 system motion 实现前，`$H` 必须返回明确的 `homing-unavailable` 或进入受控测试状态，
   不得返回完成成功。

验收标准：

- `$H` 不进入 `execute_line()`，不会产生 `unsupported-command`。
- Planner 或 SD 作业活跃时，`$H` 返回 `error:homing-busy`。
- 同时重复发送 `$H` 时仅第一条可接受。
- 归零尚未完成时，`$POS/STATUS` 不会将坐标标记为 `known`。

结束改动标准：

- 协议层、归零服务和普通 G-code 的所有权边界明确。
- 未接入限位硬件时不会误报归零成功。

当前实现状态：已新增 `homing/homing_service` 的状态与启动接口，`protocol` 已将 `$H`
从 `execute_line()` 前分流。阶段二、三的限位与 system-motion 后端尚未存在，因此空闲时
`$H` 返回 `error:homing-unavailable`；Planner 有运动、SD 作业活跃或未来归零任务运行时
返回 `error:homing-busy`。当前实现不会改变激光、脉冲、Planner 或位置可信状态。

### 阶段二：限位驱动与电平确认

实现内容：

1. 在 `driver` 增加 RayCube X/Y 限位输入初始化：GPIO42、GPIO2。
2. 提供去抖后的同步状态读取和 ISR 通知接口。
3. 将限位有效电平配置化，不根据注释假设常开或常闭。
4. 正常工作状态下，限位触发必须走安全停止入口；归零状态下只通知对应归零轴。

验收标准：

- `$H` 诊断或专用状态输出能正确显示 X/Y 未触发、手动触发、释放三种状态。
- 抖动不会被识别为一次稳定触发。
- GPIO42 与 GPIO2 的有效电平已由实机确认并固化到 machine 配置。
- 限位 ISR 不输出日志、不等待、不访问文件系统或 Planner。

结束改动标准：

- 软件能够可靠区分每个轴的触发状态。
- 未确认有效电平时不进入寻零运动阶段。

当前实现状态：已增加 `driver/limit_switch_driver`，以 GPIO42（X）和 GPIO2（Y）输入、内部上拉、
低电平触发配置运行。任意边沿 ISR 仅唤醒 driver 去抖任务；任务在 5 ms 后更新原子快照。
`$H/STATUS` 输出 `limit=(x=0|1,y=0|1)` 和去抖次数，供实机确认有效电平。该阶段尚未把限位
接到硬限位停机或 `$H` system-motion，故不会因手动触发而影响当前运动。

### 阶段三：单轴 system-motion 与限位停止

实现内容：

1. 在 stepper/driver 边界定义归零专用的定速单轴 system-motion 接口。
2. 该接口复用现有 GPTimer/RMT STEP 输出，但不创建普通 Planner block，不进入 junction 前瞻。
3. 限位 ISR 触发时停止 GPTimer 后续脉冲与 RMT 输出，并通知 homing task。
4. 为 seek、pull-off 与 locate 提供有限最大行程和超时，防止传感器失效时撞机。

验收标准：

- 固定低速单轴测试的实际 STEP 数与诊断一致。
- 人工触发对应限位时，该轴在受控距离内停止，另一轴未被意外驱动。
- 未触发限位直到最大寻程时停止并报告 `seek-timeout`。
- system-motion 期间不出现 Planner block、segment underflow、RMT retrigger 或看门狗复位。

结束改动标准：

- 单轴可安全完成“向开关移动并在触发时停止”的最小闭环。

### 阶段四：X/Y 两次定位与回退

实现内容：

1. 对每个轴顺序执行：快速 seek、退离、慢速 locate、最终退离。
2. 首版按 X 后 Y 执行，避免多轴限位抢占与单轴停止语义。
3. 每次触发后验证开关能在回退距离内释放；未释放即失败。
4. 每轴独立记录失败原因：启动时已触发、seek 超时、pull-off 未释放、locate 超时、reset。

验收标准：

- 从工作幅面中部启动时，X 与 Y 都能找到各自负向限位。
- 最终物理位置均离开开关，手动读取为未触发。
- 快速与慢速两次触发均有状态记录。
- 任一轴失败时不写入 `(0,0)`，并关闭激光。

结束改动标准：

- 两个轴的机械寻零过程可重复执行，且不会撞击限位后持续推压。

### 阶段五：提交逻辑零点与恢复普通运动

实现内容：

1. 仅在 X/Y 都完成最终回退后，调用 `stepper_set_machine_position_mm({0,0})`。
2. 在 protocol 持有 ParserState 锁的情况下同步 parser 的计划位置。
3. 清理 system-motion 临时状态，恢复普通 Planner 运动入口。
4. 在 `$POS/STATUS` 中区分“物理已回退”和“逻辑原点已提交”的完成状态。

验收标准：

- `$H` 完成后 `$POS/STATUS` 显示 `state=known`、`machine=(0.0000,0.0000)`、
  `planned=(0.0000,0.0000)`。
- 后续 `G90` + `G1 X20 Y10 F600` 从逻辑零点向幅面内部运动，位置和步数符合预期。
- `G1 X131` 或 `G1 Y111` 被现有幅面校验拒绝。
- reset、限位失败或中途取消后，`$POS/STATUS` 保持 `invalidated`，必须重新 `$H` 才可运动。

结束改动标准：

- `$H` 成功是唯一将启动后未知/失效位置恢复为可信机床坐标的路径。
- 软限位、绝对坐标、SD 雕刻和手动运动均使用同一 `(0,0)` 基准。

## 阶段三当前实现状态

已接入仅用于实机验收的 `$H/TEST=X` 与 `$H/TEST=Y`。命令分别以固定 seek
速度沿对应轴配置的物理寻限位方向运动；命中本轴限位后由 ISR 立即停止 GPTimer/RMT 脉冲并通知
归零任务完成清理。单轴测试不会写入逻辑原点，位置保持 `invalidated`；未触发限位而
走满 `1.1 * max_travel` 时进入 `faulted`，用于验证传感器失效时不会持续撞机。

归零 seek 的物理方向由 `machine::AxisHardware::homing_seek_positive_direction`
独立定义，不从普通 G-code 坐标正负推导。当前 X 轴已按实机结果设为正方向，Y 轴保持
原负方向，待其单轴测试确认后再固化。

实机验收命令：

```text
$H/TEST=X
$H/STATUS
$POS/STATUS

$H/TEST=Y
$H/STATUS
$POS/STATUS
```

验收标准：设备从对应限位外、幅面中部开始。人工触发对应开关时，仅该轴立即停止；
`$H/STATUS` 显示 `completed`，`$POS/STATUS` 显示 `state=invalidated`。若不触发开关，
受控行程结束后 `$H/STATUS` 显示 `faulted`，且不写入 `(0,0)`。

实机验收结果：X 轴 `$H/TEST=X` 已按配置的物理正方向到达 X 限位。停止后
`$H/STATUS` 返回 `state=completed|limit=(x=1,y=0)`，单轴过程没有写入逻辑原点。

## 当前非目标

- G28/G30/G92/G10 等扩展坐标命令。
- 双轴并行归零与自动方正。
- 完整 Grbl Alarm、Feed Hold、Safety Door 和实时命令集。
- 多个限位开关、传感器失效冗余或高级自诊断。
- 在普通运动中直接以限位 GPIO 替代软件幅面检查。

## 关联文件

- `machine/raycube_machine.hpp`
- `machine/raycube_machine.cpp`
- `driver/step_timer_driver.*`
- `driver/rmt_step_pulse_driver.*`
- `stepper/stepper_driver.*`
- `stepper/timer_stepper_executor.*`
- `planner/planner.*`
- `protocol/command_dispatch.cpp`
- `D:\ray2-main\components\longerlib_IDF\src\longerLib\project_extensions\grbl\machine\RayCube\GRBLSettings.h`
- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Limits.cpp`
