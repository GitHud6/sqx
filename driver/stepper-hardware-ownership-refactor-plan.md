# Stepper Hardware Ownership Refactor Plan

## 目标分析

当前工程把 ESP32-S3 的 RMT 与 GPTimer 操作放在 `stepper/` 中：

- `rmt_stepper_driver.cpp` 同时拥有 RMT channel、同步管理器、RMT symbol 编码、
  stream task、segment 队列、Bresenham 和实际位置累计。
- `timer_stepper_executor.cpp` 同时拥有 GPTimer、RMT LL 寄存器、GPIO matrix、
  STEP 脉冲触发、Bresenham 和轴使能。

这使硬件所有权和运动语义混在一起。同一组 X/Y STEP GPIO 存在两套 RMT 后端，
`planner` 与 `protocol` 还直接依赖 `rmt_stepper_*` 接口。后续替换 RMT 实现、增加
其他 step 输出外设或定位时序问题时，影响范围会扩散到 planner、stepper 与协议状态输出。

本计划的目标不是立即解决连续 segment 供料问题，也不是改变已验证的阶段二短线
执行语义；目标是先建立唯一、可验证的硬件边界：

```text
planner
  -> stepper execution semantics
       -> driver step timer / RMT step pulse capability
            -> ESP-IDF / ESP32-S3 peripherals
```

## 目标职责

### driver

`driver` 是片上外设的唯一所有者。

- `step_timer_driver`：拥有 GPTimer 生命周期、ISR 注册、启动、停止和定时周期。
- `rmt_step_pulse_driver`：拥有 RMT channel、GPIO matrix、RMT memory、脉冲波形和
  `trigger(axis)`。
- 驱动只接受引脚、周期、轴号等硬件参数；不理解 G-code、PlannerBlock、StepSegment、
  位置、Bresenham、激光、Feed Hold 或 SD 作业。
- RMT 只能有一个驱动实例拥有 X/Y STEP GPIO。禁止 RMT high-level driver 和 RMT LL
  寄存器后端同时绑定同一组 STEP GPIO。

### stepper

`stepper` 只拥有机床步进执行语义。

- 执行器状态、Bresenham、DIR/ENABLE、实际步数累计和 segment 消费。
- timer ISR 中按确定的步进状态调用 `rmt_step_pulse_trigger(axis)`，而不是直接访问
  `RMT`、`rmt_ll_*`、`RMT.chndata` 或 GPIO matrix。
- 通过 `step_timer_driver` 注册 ISR，而不是直接创建或停止 GPTimer。
- 不创建 RMT transaction stream task，不保存 RMT channel handle，不调用
  `rmt_transmit()`。

### planner 与 protocol

- `planner` 只管理 block queue、前瞻、segment 生成、完成语义、Hold 和 Reset。
- `planner` 不包含 `rmt_stepper_driver.hpp`，不直接调用 RMT 接口。
- `protocol` 的 `$POS/STATUS` 只报告稳定的执行状态，例如 planner queue、实际位置、
  stepper 执行状态；不暴露某个 RMT 后端的 transaction 计数。

## 不纳入本计划

- 修复当前连续 segment ring 的补料、下溢或 block 完成策略。
- 改变 LightBurn、SD、gzip、G-code 解析和 `ok` 协议行为。
- 引入新运动学、限位、回零或激光功率同步逻辑。
- 在没有实机验收前删除已验证的阶段二短线执行路径。

## 阶段一：登记现有所有权与冻结双后端（已完成：文档登记）

实现步骤：

1. 在 `driver/README.md` 登记 GPTimer 与 RMT 的当前引脚、时钟、API 类型和唯一所有者。
2. 记录 `rmt_stepper_driver` 与 `timer_stepper_executor` 对 RMT、GPTimer、STEP GPIO 的
   当前访问点。
3. 明确当前实际执行路径只能选择一个后端；另一个后端只能保留为未调用的迁移参考，
   不得在启动时初始化或绑定 STEP GPIO。
4. 为后续稳定状态输出定义与具体 RMT transaction 无关的最小诊断字段。

验收标准：

- 可以从文档确认 X/Y STEP GPIO、RMT channel 与 GPTimer 的唯一运行时所有者。
- 全工程中不存在两个启动路径同时初始化或绑定同一组 STEP GPIO。
- 不修改电机实际输出、Planner 行为或 G-code 行为。

结束改动标准：

- 形成后续迁移使用的外设所有权表。
- 本阶段仅完成文档与依赖清点，不做实机行为变化。

阶段结果：

- `driver/README.md` 已登记当前活动路径、X/Y STEP GPIO、RMT channel、GPTimer 配置与旧
  RMT stream 的冻结规则。
- 本阶段未修改运行代码，因此不要求新增实机验收；当前短线执行行为保持原样，不能据此
  声称已完成后续硬件驱动迁移。

## 阶段二：提取 RMT 单脉冲硬件驱动（代码已实现，待实机验收）

实现步骤：

1. 新建 `driver/rmt_step_pulse_driver.hpp/.cpp`。
2. 将 RMT 时钟、memory、波形、GPIO matrix、寄存器或 ESP-IDF RMT API 全部迁入该文件。
3. 对外只提供初始化、停止、绑定状态查询和 ISR 可调用的单脉冲触发接口。
4. RMT 配置使用明确的波形结构或配置常量，不允许 `stepper` 拼接 RMT symbol 或直接写
   `RMT.chndata`。
5. 迁移后由当前已验证短线执行路径调用新 driver；旧 RMT stream 不得初始化。

验收标准：

- 活动执行路径 `stepper/timer_stepper_executor.cpp` 中不再包含
  `driver/rmt_*.h`、`hal/rmt_ll.h`、`soc/rmt_struct.h` 或
  `esp_rom_gpio_connect_out_signal()`；冻结的旧 `rmt_stepper_driver.cpp` 留待阶段四清理。
- X/Y 正反向短线仍可实机运动，STEP 空闲电平和脉冲宽度满足硬件要求。
- 冷启动、停止与异常退出后 STEP 均回到安全空闲电平。

结束改动标准：

- 活动单脉冲路径的 RMT 初始化与 GPIO matrix 绑定只存在于
  `driver/rmt_step_pulse_driver.cpp`；旧 stream 后端仍被冻结，必须在阶段四删除。
- 实机验收通过后提交本阶段改动，再进入阶段三。

当前实现状态：

- 已新增 `driver/rmt_step_pulse_driver.hpp/.cpp`，并迁入 RMT LL、GPIO matrix、固定
  脉冲符号、初始化、停止和 ISR 单脉冲触发。
- `timer_stepper_executor` 不再直接包含或访问 RMT LL、RMT 寄存器和 GPIO matrix；它保留
  GPTimer、Bresenham、DIR/ENABLE 与位置累计，并通过 driver 接口触发脉冲。
- 尚未在此阶段版本上完成实机短线验收，因此不得提交阶段二，也不得进入阶段三。

## 阶段三：提取 GPTimer 硬件驱动（代码已实现，待实机验收）

实现步骤：

1. 新建 `driver/step_timer_driver.hpp/.cpp`。
2. 将 GPTimer 创建、enable、alarm 配置、start、stop 和 ISR 注册迁入该文件。
3. 定义小而固定的 ISR callback 入口；timer driver 只负责按周期调用，不读取 Planner 或
   segment ring。
4. `stepper` 保存运动状态，并通过该 callback 执行 Bresenham 与 RMT 单脉冲触发。
5. Reset/Stop 先停止 timer，再由 stepper 关闭轴和清理执行状态。

验收标准：

- `stepper/` 中不再包含 `driver/gptimer.h`，不调用 `gptimer_*`。
- 单轴、XY、正反向短线的步数与 `$POS/STATUS` 一致。
- 连续短线不会发生 watchdog、重复脉冲或 position invalidation。

结束改动标准：

- GPTimer 只有 `driver/step_timer_driver.cpp` 一个所有者。
- 实机验收通过后提交本阶段改动，再进入阶段四。

当前实现状态：

- 已新增 `driver/step_timer_driver.hpp/.cpp`，拥有 GPTimer 创建、回调注册、1 MHz
  周期配置、启动和停止。
- `timer_stepper_executor` 不再包含 `driver/gptimer.h` 或调用 `gptimer_*`；它注册固定
  ISR 回调并保留 Bresenham、DIR/ENABLE、实际位置和完成通知。
- 尚未完成阶段三实机验收，因此不得提交本阶段或进入阶段四。

## 阶段四：清理旧 RMT stream 后端与上层泄漏（代码已实现，待实机验收）

实现步骤：

1. 确认旧 `rmt_stepper_driver` 中哪些内容已被新 driver/stepper 路径覆盖。
2. 删除或拆分其中的 RMT transaction、stream task、RMT diagnostics 和旧回退接口；不保留
   会再次绑定 STEP GPIO 的死代码。
3. 从 `planner` 删除 `rmt_stepper_*` 依赖，改为调用 stepper 的通用执行状态接口。
4. 从 `protocol` 删除 RMT transaction 专属状态展示，保留稳定的位置、队列和执行状态。
5. 更新 `driver/README.md` 与 `stepper/README.md`，说明最终边界。

验收标准：

- 工程中只存在一套 X/Y STEP 的 RMT 初始化和输出路径。
- `planner`、`protocol` 不再 include RMT 后端头文件。
- 不存在未调用但能初始化 RMT 并接管 STEP GPIO 的旧后端。

结束改动标准：

- 清理后构建通过。
- 已验证的短线测试与 Reset 安全输出均通过实机验收。
- 实机验收通过后提交本阶段改动；此后才开始重新处理连续 segment 执行问题。

当前实现状态：

- 已删除 `stepper/rmt_stepper_driver.hpp/.cpp` 及其 ESP-IDF high-level RMT TX、transaction
  queue、stream task 和回退接口，并从构建目标移除。
- planner 已删除旧 stream 的预填充、completion queue、RMT 回退和 RMT 专属 Hold/Reset 调用；
  当前只经 `timer_stepper_executor` 执行 Motion block。
- protocol 的 `$POS/STATUS` 已移除 RMT transaction 字段，只保留位置、规划位置和 planner
  队列/完成计数。
- 尚未完成实机验收，因此不得提交阶段四或开始连续 segment 供料改造。

## 迁移原则

1. 每次只迁移一个外设所有权，不同时重写 segment 流、激光同步或 Planner 前瞻。
2. 先让已验证的短线行为在新边界下保持一致，再删除旧实现。
3. ISR 热路径只能调用 IRAM 安全的 timer/RMT driver 接口；不调用日志、队列等待、文件、
   LEDC 或 Planner API。
4. 每阶段完成并实机验收后，按项目规则提交，再开始下一阶段。
