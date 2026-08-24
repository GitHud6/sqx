# Grbl Planner 与 Stepper Segment 迁移计划

## 目标分析

LightBurn 灰度层会产生大量相邻的短直线命令，例如：

```gcode
G1 X0.05 S111
G1 X0.05 S103.9
G1 X0.05 S98.4
```

在 RayCube 的 `80 steps/mm` 配置下，`X0.05` 约为 4 个 STEP。以
`F6000 mm/min` 雕刻时，速度为 `100 mm/s`，每个像素的理想持续时间约为
`500 us`。这会放大运动系统中任何逐命令同步等待、逐命令停机，或功率更新与
实际速度脱节的问题。

当前 `ray2-idf` 的 `MotionBlock` 执行器按单个 block 直接配置 RMT、等待传输完成
再处理下一个 block。该实现适合验证基础运动和激光生命周期，但不是最终运动控制
架构。它的限制是尚未迁入完整的运动规划与 step segment 执行层，不能据此把问题
归为 RMT 本身，更不应优先引入只面向灰度光栅的 `RasterRun` 特化执行器。

`ray2-main` 已具备应迁移的通用 Grbl 架构：

```text
G-code motion
  -> Planner block ring buffer
  -> look-ahead: reverse pass / forward pass
  -> junction speed + acceleration profile
  -> Stepper prep: step segment ring buffer
  -> RMT backend: timed STEP pulse generation
```

其中：

- `Planner.cpp` 的 `planner_recalculate()` 对队列中的 block 进行反向、正向前瞻。
- `plan_buffer_line()` 根据相邻路径计算 junction speed；同方向的连续短 `G1` 不应在
  每个像素边界降为零速。
- `Stepper.cpp` 的 `st_prep_buffer()` 将 planner block 转换为小时间片的 segment，并以
  独立环形缓冲把规划与实际 step 输出解耦。
- 激光 M4 的功率计算应建立在 segment 的实际速度上，而非仅根据原始 G-code 的目标 F 值。

本计划的目标是以 `ray2-main` 的 Planner / Stepper 分层为参考，逐步替换当前直接
`MotionBlock -> RMT` 的执行模型。灰度雕刻（`6000 mm/min`、`508 DPI`）是最终的压力
验收场景，而不是单独的架构分支。

## 现有模块复用与迁移接口

本次迁移只替换运动业务层的内部实现，不把 `ray2-main` 的 Arduino 全局架构、全局状态
或硬件抽象直接搬入当前 ESP-IDF 工程。当前已验证模块的职责保持稳定：

```text
保持不变
UART0 -> protocol -> command -> motion_submit() -> planner_submit()
                                                  |
machine: RayCube 引脚、步距、方向与使能电平
stepper_driver: DIR / ENABLE GPIO 唯一写入者
rmt_stepper_driver: RMT 通道和 STEP 脉冲硬件唯一拥有者
laser_driver: LEDC PWM、GPIO5 安全使能唯一拥有者

逐步替换或新增
planner: FIFO FreeRTOS Queue -> 可前瞻的 Planner block ring buffer
planner_task: block 直接执行 -> Stepper prep / segment buffer 填充
rmt_stepper_driver: execute(block) -> consume(segment)
laser_planner_adapter: block 生命周期 -> segment 实际执行边界
```

迁移接口契约：

1. `motion_submit()`、`planner_submit()` 的调用方向和 `Accepted` / `Full` / `Rejected` 语义
   保持不变。上层不感知 Planner 内部由 FreeRTOS Queue 改为 ring buffer。
2. `machine::raycube_axis()`、`stepper_set_direction()`、`stepper_set_enabled()`、RMT 通道初始化
   以及激光 PWM 初始化不迁移、不复制。业务层只能调用它们暴露的接口。
3. Planner block 只能由 `motion` 构造；segment 只能由 Stepper prep 从已规划 block 检出；RMT
   只能消费 segment。禁止 `protocol`、`command` 或 `motion` 直接发 STEP/RMT。
4. 已检出的 segment 不允许被后续前瞻重算修改。Planner 仅能重算尚未检出到 segment buffer 的
   block，确保执行中的硬件 payload 与规划状态一致。
5. RMT 后端改造期间保留现有 block 执行接口作为受控回退；新 segment 后端通过独立接口接入，
   在单轴、XY、Hold、Reset 验收完成后再替换默认路径。
6. 激光驱动仍由 `laser_driver` 独占。适配层只接收 segment 生命周期和实际速度，不能在 G-code
   解析、block 入队或前瞻计算时直接改变 GPIO4 / GPIO5。
7. `M5`、Hold、Reset、硬件执行失败仍是高优先级安全路径：先由 `laser_driver` 禁用 GPIO5，
   再停止或取消尚未开始的 segment，最后禁用参与轴。

## 非目标与边界

- 不改变 `UART0 -> protocol -> command -> motion` 的命令接收和解析职责。
- 不改变已验证的 RayCube STEP、DIR、ENABLE 及激光 PWM 引脚配置。
- 不把尚未接受的 UART 命令提前当作已规划运动。
- 不在本计划中实现 G2/G3、限位、Homing、完整 Alarm 状态机或网络/SD 流控。
- 在通用 Planner / Segment 执行路径完成前，不实现仅适配单轴 X 灰度行的 RasterRun。
- 每完成一个阶段并通过对应实机验收后，开始下一阶段前按项目规则提交本阶段改动。

## 实现步骤与验收标准

### 阶段一：定义 Planner block 与环形队列

实现内容：

1. 以 `ray2-main` 的 `plan_block_t` 为参考，在当前 Planner 中定义独立于 RMT payload 的
   规划 block：目标步数、各轴步数、方向、距离、进给、加速度、激光/主轴模式、S 值及
   `execution_id`。
2. 在 `planner_submit()` 接口不变的前提下，将 Planner 内部的 FIFO FreeRTOS Queue 改为固定
   容量的 planner block ring buffer，明确 head、tail、planned 的所有权和并发规则。
3. 保持 `motion` 只负责将已解析的直线请求转换为 planner block 并入队；它不等待 RMT，且不
   直接调用 `stepper`、`rmt_stepper_driver` 或 `laser_driver`。
4. 队列满时采用确定的背压策略，不能覆盖未执行 block，也不能错误回复 `ok`。

验收标准：

- 连续发送多条 `G1` 后，block 的步数、方向、F、S、`execution_id` 和入队顺序可观测且正确。
- 队列满、空队列、零步命令和 M5 等非运动事件均有确定结果，不丢失或重排命令。
- 原有 `motion_submit()`、`planner_submit()` 和协议侧 `ok` / `Busy` 行为无需修改调用方即可通过
  回归验证。
- 此阶段不要求连续运动；既有单条直线和激光安全关闭能力不回归。

阶段状态：代码已完成，待构建与实机验收。

### 阶段二：迁移 junction speed 与前瞻重算

实现内容：

1. 从 `ray2-main` 迁移连续 block 的单位向量、最大 junction speed 和轴加速度限制计算。
2. 实现 reverse pass 与 forward pass，计算每个 block 的允许 entry / exit speed。
3. 新 block 入队、Feed Hold 后重规划、队列清空等情形必须重新建立一致的速度边界。
4. 初版只支持 G0/G1 的 Cartesian XY 直线；不支持的运动类型保持显式拒绝或现有回退。

验收标准：

- 多个同方向连续 `G1 X0.05 F6000` 的相邻 junction 具有非零允许速度。
- 方向改变或折角时，junction speed 被限制在安全值；首 block 和末 block 保持从/到零速的边界。
- 改变队列深度不会改变已确定的步数、方向、命令顺序或最终坐标。

阶段状态：代码已完成，待构建与实机验收。

### 阶段三：生成加减速 profile 与 step segment

实现内容：

1. 以 `ray2-main` 的 `st_prep_buffer()` 为参考，建立 Stepper prep 层，从 Planner 队列读取
   已规划 block。
2. 按 entry speed、nominal speed、exit speed、加速度和距离生成加速、匀速、减速 profile。
3. 将 profile 切分为固定时间预算的小 step segment，建立独立于 Planner 的 segment ring buffer。
4. segment 从 Planner block 检出后，保留足够的执行数据，使已检出的 segment 不再受后续
   planner 重算影响。
5. segment 使用独立的数据结构和生命周期；不将 `MotionBlock` 或 RMT symbol 数组作为跨层共享
   的可变对象。

验收标准：

- segment 的总步数严格等于其来源 planner block 的步数，XY 比例和方向正确。
- segment buffer 在正常持续运行时保持预填充，Planner 重算不改写已检出的 segment。
- 单条长线可观测到加速、匀速、减速；同向短线跨 block 时速度连续。
- Planner task 不再因同步等待单条长运动而触发 watchdog。

### 阶段四：将 RMT 改为 segment 执行后端

实现内容：

1. 在保留既有 `rmt_stepper_execute()` / `rmt_stepper_execute_xy()` 回退接口的前提下，新增 RMT
   segment 消费接口；新后端仅消费已生成的 step segment，不再将原始 G-code / MotionBlock 视为
   一次完整 RMT 事务。
2. 保证 RMT 传输 payload 的内存生命周期覆盖硬件消费期，并用受控的 segment 队列或 refill
   机制维持输出。
3. 在 segment buffer 有余量时持续供给 RMT，避免每个短 Planner block 调用
   `rmt_tx_wait_all_done()`、关闭电机和重新配置 GPIO。
4. 定义 Hold、Reset、执行错误和队列耗尽时的安全停机：先关闭激光使能，再终止或排空允许的
   step 输出，最后关闭电机。

验收标准：

- 多条同方向短 G1 的 STEP 波形在跨 block 边界不出现软件引入的停顿。
- 长时间连续运动不触发 watchdog，不耗尽堆栈，不引用失效 payload。
- Hold、Reset、RMT 错误和正常结束均能停止输出并恢复到可接受新命令的状态。
- 普通单轴及 XY 直线的终点步数、方向和 ENABLE 行为正确。
- 旧 block 执行回退接口仍可完成已验证的单条直线；segment 默认路径只在对应实机验收后启用。

### 阶段五：将激光功率同步到 segment

实现内容：

1. 将 Planner block 中的主轴模式和 S 值复制到来源 segment，并将现有
   `laser_planner_adapter` 从 block 完成即关闭的语义调整为 segment 生命周期语义，确保功率
   更新发生在对应运动实际开始之前，而不是命令解析或 block 入队时。
2. 按 `ray2-main` 的 M4 思路，基于 segment 实际速度与 programmed rate 的比例实现动态功率
   补偿；M3 保持固定功率语义。
3. 测量 LEDC duty 更新到 GPIO4 生效的延迟与抖动，确认其是否满足 `500 us` 像素周期。
4. 若 LEDC 不满足实测要求，再单独规划激光硬件时序后端；这不是本阶段的预设实现。

验收标准：

- 每个 segment 的实际 PWM 对应其来源 S 值，无提前或滞后一段的问题。
- M4 在加速、减速和 Feed Hold 恢复时按实际速度调整功率，M3 不发生该补偿。
- M5、Hold、Reset、取消和运动结束时，GPIO5 及时禁用且无残留出光。

### 阶段六：灰度雕刻端到端验收

实现内容：

1. 使用同一份 LightBurn 灰度文件、`6000 mm/min`、`508 DPI` 参数进行连续输入测试。
2. 记录 UART 接收/`ok` 节奏、Planner block 深度、segment buffer 深度、RMT 供给状态、STEP
   间隙、PWM 更新与任务 watchdog 状态。
3. 使用逻辑分析仪采集 X STEP、激光 PWM、激光 ENABLE，区分运动、功率、电气与材料造成的
   图案问题。

验收标准：

- 连续 `G1 X0.05 S...` 灰度行不再因逐 block 同步执行形成固定软件停顿。
- 灰度 S 序列、总步数、行末 M5 和最终坐标均与输入一致。
- UART 不断供，Planner 与 RMT 不触发 watchdog，Hold / Reset 安全语义不回归。
- 若仍有条纹，可由测量明确归因到机械、材料、PWM 电气或剩余的时序限制。

## 完成标准

- Planner、前瞻、junction、加减速 profile、step segment buffer 和 RMT 后端职责明确且已打通。
- RMT 不再承担逐 G-code block 同步执行的调度职责。
- 激光功率在实际 segment 执行边界更新，并为 M4 动态功率补偿提供真实速度基础。
- 灰度光栅在目标参数下通过连续性、顺序、吞吐、watchdog 和安全停机的实机验收。

## 参考实现

- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Planner.cpp`
- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Planner.h`
- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Stepper.cpp`
- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\Stepper.h`
- `D:\ray2-main\components\Grbl_Esp32_Source\Grbl_Esp32\src\MotionControl.cpp`
