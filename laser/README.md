# 激光模块

`laser` 是 RayCube 激光输出的唯一所有者。它负责 LEDC PWM、激光使能 GPIO 与安全关闭，
不解析 G-code，不参与运动规划，也不直接控制步进电机。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `laser_driver.hpp/.cpp` | 初始化并操作 LEDC/GPIO；提供普通任务和 ISR 安全的功率更新、关闭与状态查询接口。 |
| `laser_planner_adapter.hpp/.cpp` | 订阅 planner 生命周期事件，维护运动期间的激光输出许可，并在 Hold、取消、失败或队列排空时关闭输出。 |

## 运行链路

```text
M3 / M4 + S
  -> command / motion / planner block
  -> stepper_prep 生成 StepSegment 的 laser_power_per_mille
  -> GPTimer 切换实际执行的 segment
  -> laser_apply_segment_power_from_isr()
  -> LEDC PWM + 激光 Enable GPIO
```

激光功率只在对应运动 segment 即将实际执行时更新。G-code 被解析、block 入队或 planner
进行前瞻时均不得直接操作激光 GPIO。

## 功率语义

- `M3`：每个 segment 使用来源 S 值对应的固定功率。
- `M4`：`stepper_prep` 根据 segment 实际速度与 programmed feed 的比例缩放来源 S 值；
  低速加减速段相应降低功率。
- `S0` 或 segment 功率为零：关闭激光输出。
- 激光模式为 Off 的运动：关闭激光输出。

`laser_driver` 接受的普通任务功率范围为 `0..machine::raycube_laser().max_power`。无效数值、
超范围数值或 LEDC/GPIO 更新失败都会请求安全关闭。

## 硬件配置

硬件参数由 `machine/raycube_machine.cpp` 的 `LaserHardware` 定义，当前 RayCube 为：

| 项目 | 当前值 |
| --- | --- |
| PWM 输出 | GPIO4 |
| 激光使能 | GPIO5，高有效 |
| PWM 极性 | 非反相 |
| PWM 频率 | 2 kHz |
| LEDC 分辨率 | 10 bit，duty `0..1023` |
| 最大 S 值 | `1000` |

禁止在本模块硬编码机型引脚或最大功率；这些值必须从 `machine::raycube_laser()` 取得。

## 安全与并发约束

- 初始化必须处于关闭状态：先关闭 Enable，再停止 PWM。
- `laser_disable()` 可重复调用；它关闭 GPIO5、停止 LEDC，并清除输出状态和 duty 记录。
- `laser_disable_from_isr()` 与 `laser_apply_segment_power_from_isr()` 只能调用 LL 寄存器接口，
  不得在 ISR 中调用会阻塞的 ESP-IDF 高层驱动。
- Feed Hold 会撤销 `motion_output_permitted` 并立即关闭输出；Resume 只恢复许可，直到下一
  segment 实际开始才可能重新出光。
- block 取消、执行失败和队列排空均必须关闭输出。

## 模块边界

上游依赖：

- `machine`：提供 `LaserHardware`。
- `planner`：提供生命周期事件。
- `stepper_prep`：在非 ISR 上计算每个 segment 的功率。
- `timer_stepper_executor`：在 segment 边界调用 ISR 功率接口。

下游依赖：ESP-IDF 的 GPIO、LEDC 与 HAL LL 接口。

本模块不得反向依赖 `command`、`protocol`，不得读取或修改 planner 队列、junction、坐标、
STEP 或 DIR 信号。

## 验收重点

- 上电和初始化后 GPIO5 为关闭电平，PWM duty 为零。
- `M3 S1000` 配合线性运动能够出光；`M5` 后输出关闭。
- `M4` 灰度雕刻时，功率与实际 segment 速度同步更新。
- Hold、Reset、限位归零、planner 失败或 segment 供料异常后，激光不会保持输出。
