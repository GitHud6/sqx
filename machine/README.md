# 机型配置模块

`machine` 是当前固件的机型硬件抽象层。它只保存 RayCube 的静态硬件参数和运动限制，
为上层运动、归零、步进、限位和激光模块提供统一查询接口。

本模块不初始化 GPIO、LEDC、RMT 或 GPTimer，不产生运动，也不解析 G-code。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `raycube_machine.hpp` | 定义轴、激光硬件配置结构、轴枚举和公共常量。 |
| `raycube_machine.cpp` | 保存当前 RayCube 的静态配置，并实现配置查询接口。 |

## 当前机型

当前仅支持 RayCube，配置来源为 `raycube_machine.cpp` 中的编译期常量。

### X/Y 轴

| 项目 | X | Y |
| --- | ---: | ---: |
| STEP | GPIO15 | GPIO8 |
| DIR | GPIO7 | GPIO17 |
| DISABLE | GPIO16 | GPIO18 |
| 限位输入 | GPIO42 | GPIO2 |
| steps/mm | 80 | 80 |
| 最大速度 | 12000 mm/min | 12000 mm/min |
| 加速度 | 500 mm/s² | 500 mm/s² |
| 最大行程 | 130 mm | 110 mm |
| DIR 反相 | 是 | 是 |
| DISABLE 有效电平 | 高 | 高 |
| 限位有效电平 | 低 | 低 |

当前 X/Y 的物理归零寻限位方向均由 `homing_seek_positive_direction` 独立定义为负方向。
该字段描述物理运动方向，不应从普通 G-code 的正负方向推导。

### 激光

| 项目 | 当前值 |
| --- | ---: |
| PWM 输出 | GPIO4 |
| Enable | GPIO5，高有效 |
| PWM 极性 | 非反相 |
| PWM 频率 | 2 kHz |
| 最大功率值 | 1000 |

激光的实际 PWM 和安全关闭由 `laser` 模块负责；`machine` 只提供上述参数。

## 对外接口

```cpp
const AxisHardware& raycube_axis(Axis axis);
const LaserHardware& raycube_laser();
```

公共常量：

- `kAxisCount`：当前为 2，表示 X/Y。
- `kStepPulseMicroseconds`：STEP 高电平宽度，当前为 3 µs。

调用方必须通过 `raycube_axis()` 和 `raycube_laser()` 读取配置，不应复制引脚、步距、
幅面或功率常量。

## 模块联动

```text
machine
  ├─ motion       读取步距、最大速度和幅面限制
  ├─ planner      使用轴加速度和运动限制
  ├─ stepper      配置 STEP / DIR / DISABLE 并换算实际位置
  ├─ driver       配置限位 GPIO、RMT 和定时器相关硬件
  ├─ homing       读取限位输入和归零寻向
  └─ laser        读取 PWM / Enable 配置
```

各业务模块可以读取机型配置，但 `machine` 不得反向依赖这些模块，避免形成硬件配置与
业务执行的循环依赖。

## 方向与坐标约束

- `direction_inverted` 只描述 DIR 电平与逻辑方向之间的映射。
- `homing_seek_positive_direction` 只描述归零时的物理寻限位方向。
- `max_travel_mm` 用于软限位和目标坐标校验，当前幅面为 X=`0..130` mm、Y=`0..110` mm。
- `steps_per_mm` 用于 mm 与开环 STEP 计数之间的换算。
- `machine` 不维护当前位置；实际位置由步进执行路径在 STEP 已发出后累计。

## 修改规则

修改机型参数前必须确认：

1. 引脚、电平和方向已与目标硬件原理图及实机结果一致。
2. 步距、最大行程、速度和加速度与运动规划及驱动器配置匹配。
3. 激光 PWM/Enable 参数与激光模块的安全关闭逻辑一致。
4. 需要支持新机型时，优先新增独立机型配置，不直接污染 RayCube 常量。
5. 修改后至少验证初始化、归零、边界运动、方向和激光安全关闭行为。

## 当前范围

- 已实现：RayCube X/Y Cartesian 静态硬件配置。
- 已实现：RayCube 激光 PWM 与 Enable 参数配置。
- 未实现：运行时切换机型、通用多机型注册表和 Z 轴配置。
