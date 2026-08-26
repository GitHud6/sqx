# 步进运动执行模块

`stepper` 负责把 planner/segment 产生的运动数据转换为实际的 STEP/DIR/ENABLE 执行，
并维护基于已发出 STEP 脉冲的开环机床位置。

本目录包含步进 GPIO、位置状态、segment 生产、Timer 执行器和 AMASS 细分逻辑。RMT 与
GPTimer 的底层外设配置属于 `driver` 模块；本模块只通过 driver 接口使用它们。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `stepper_driver.hpp/.cpp` | 初始化 STEP/DIR/ENABLE GPIO，设置方向和使能，维护实际执行步数与位置可信度。 |
| `stepper_prep.hpp/.cpp` | 将已规划的 `PlannerBlock` 按速度 profile 拆成固定容量的 `StepSegment` ring。 |
| `timer_stepper_executor.hpp/.cpp` | 使用 step timer 的 ISR 消费 segment，执行 DDA/AMASS，触发 RMT 单脉冲并发布完成事件。 |

## 运行链路

```text
planner PlannerBlock
        |
        v
stepper_prep
  - 梯形/三角形速度 profile
  - StepSegment
  - 激光功率与 AMASS 元数据
        |
        v
SPSC StepSegment ring
        |
        v
GPTimer ISR
  - DDA / AMASS 步数分配
  - DIR 更新
  - 触发 RMT 单 STEP 脉冲
  - 累计实际执行位置
        |
        v
driver: step timer + RMT + GPIO
```

任务上下文负责生产 segment 和处理完成通知；ISR 只消费已经发布的不可变 segment，不能
访问 planner、文件系统、日志、LEDC 或阻塞 API。

## STEP/DIR/ENABLE 所有权

`stepper_driver` 是 STEP、DIR、ENABLE GPIO 的唯一业务层写入者：

- `stepper_init()` 配置 RayCube 轴 GPIO，STEP 初始为低电平，轴初始为禁用；
- `stepper_set_direction()` 和 ISR 版本负责逻辑方向到实际 DIR 电平的映射；
- `stepper_set_enabled()` 负责单轴使能和关闭；
- 普通 `stepper_pulse()` 只作为同步/兼容路径，连续运动使用 Timer ISR + RMT 后端。

底层 RMT 通道和 Timer 外设不得在本模块重复初始化。具体 GPIO 编号、电平和脉冲宽度均从
`machine::raycube_axis()` 读取。

## 开环位置模型

设备没有编码器，因此位置是根据实际执行路径累计的开环模型：

```text
实际发出 STEP
  -> 按当前 DIR 累计 machine_position_steps
  -> steps / steps_per_mm
  -> machine_position_mm
```

位置状态为：

- `Unknown`：尚未建立可信原点；
- `Known`：已通过归零或明确位置设置建立基准；
- `Invalidated`：Reset、执行失败、下溢或取消后不能再相信当前位置。

规划目标位置不得直接覆盖实际位置。`stepper_record_executed_steps_from_isr()` 只能在
对应 STEP 已由执行器触发后调用。Reset 或硬件执行异常后，必须重新通过 `$H` 或明确的
位置设置建立坐标基准。

## StepSegment ring

`stepper_prep` 使用固定容量的 SPSC ring：

- 容量：64 个 segment；
- 目标单段时间：约 750 µs；
- 低水位：16 个；
- 生产者：planner task；
- 消费者：Timer ISR；
- segment 发布后不可修改，消费完成后才能推进 tail。

发布使用 release 语义，消费使用 acquire 语义。ISR 不使用 mutex、FreeRTOS queue、动态
内存或日志。低水位只通知任务补料；ring 为空时属于真实下溢，必须停止 Timer、停止后续
脉冲、发布故障完成事件并进入安全状态。

## Segment 生成

`stepper_prep_start_block()` 从 PlannerBlock 的入口速度、出口速度、目标速度、加速度和
路径长度计算梯形或三角形 profile。`stepper_prep_fill()` 按主轴步数和累计距离生成 segment，
每个 segment 固化：

- X/Y 步数和方向；
- duration 与原始 step interval；
- 起止速度和加减速阶段；
- 激光功率、主轴模式和 execution id；
- 是否为 block 的最后一个 segment。

stepper_prep 只消费 planner 已确定的 profile，不重新计算 junction，也不访问硬件。

## AMASS 与 DDA

对于多轴低步频 segment，AMASS 根据原始 block 的多轴步数和 step interval 选择 0~3 级。
Timer executor 将一个 segment 的执行 tick 扩展为 `2^level` 倍，并使用每轴误差累积的
统一 DDA 分散 STEP：

```text
每个 timer tick:
  axis_error[axis] += axis_steps[axis]
  error >= execution_ticks -> 触发该轴一个 STEP
```

AMASS 不改变 X/Y 的总步数、方向、路径长度或最终坐标，只改善低频多轴脉冲的时间分布。
单轴运动保持 `amass=0`。实际 Timer 周期为原 segment interval 除以 `2^level`，并受
STEP 安全最小周期限制。

## Timer 执行器

`timer_stepper_executor` 是非阻塞 segment 执行器：

1. 任务从 segment ring 装载首段并设置方向、使能和激光 segment 状态。
2. 启动 step timer 后立即返回，不等待整个 block。
3. Timer ISR 执行 DDA/AMASS，并调用 driver 的 RMT 单脉冲触发接口。
4. 当前 segment 完成时，ISR 直接消费下一 segment，避免任务调度造成段间空洞。
5. block 的最后 segment 完成后写入 completion ring，通知 planner task 回收 block。

RMT 只负责固定宽度的单个 STEP 脉冲，不承担 block、segment、加速度或队列调度。

## 执行完成与异常

- segment 完成：ISR 发布 `SegmentCompletion`，planner task 后续回收对应 block；
- segment ring 下溢：停止 Timer、关闭激光并发布 faulted completion；
- Reset/Hold/执行错误：停止 Timer 和 RMT，关闭激光，必要时关闭轴并使位置失效；
- 连续运动：轴保持使能，只有 planner、segment、执行器均空闲并经过空闲延时后才关闭；
- 归零 system motion：允许按轴停止，某一限位触发后仅停止对应轴，两轴均完成后结束。

## 诊断

`stepper_segment_diagnostics()` 提供：

- segment 当前水位、容量和历史高水位；
- 下溢次数；
- 当前 execution id；
- 原始 interval、AMASS level 和实际 Timer interval；
- 当前 segment 起止速度；
- 待处理 completion 数量。

`timer_stepper_executor_diagnostics()` 提供初始化、Timer 运行状态和 Timer ISR 计数。
这些数据通过 `$POS/STATUS` 供实机观察，不应恢复逐脉冲或逐 segment 高频日志。

## 模块边界

上游依赖：`machine` 提供轴参数，`planner` 提供已完成前瞻的 block。

协作模块：

- `driver`：拥有 GPTimer、RMT 和底层外设寄存器；
- `planner`：拥有 block ring、前瞻、冻结边界和 block 完成语义；
- `laser`：拥有 PWM/Enable，并在 segment 生命周期中处理激光安全状态；
- `homing`：提交 system motion 并根据限位中断停止轴。

本模块不得解析 G-code、读取 SD/UART、执行 junction 规划、访问文件系统或直接实现 RMT
寄存器配置。

## 当前范围

- 已支持 RayCube Cartesian XY 的 G0/G1 直线执行；
- 已支持梯形/三角形 profile、连续 segment ring 和 AMASS；
- 已支持开环位置累计、归零路径、Reset/异常失效和完成通知；
- 未支持 Z 轴、G2/G3 圆弧和编码器闭环反馈。

## 验收重点

- 单轴、XY 非等比直线、连续短线、90 度转弯和反向运动的最终步数正确；
- AMASS 双轴低速运行无尖啸，且不改变目标距离；
- segment ring 正常运行无 underflow，Timer ISR 不触发 watchdog；
- segment 边界无任务调度级空洞，电机不会在连续 block 间反复使能/禁用；
- Reset、Hold、执行失败、下溢和归零限位都能停止脉冲、关闭激光并给出正确位置状态。
