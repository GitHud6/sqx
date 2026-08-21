# stepper

本模块将来拥有 STEP、DIR、DISABLE GPIO 的唯一写入权，并向 Planner 提供最小步进执行接口。

当前已实现 GPIO 初始化、使能/禁用、方向写入和单脉冲输出。正式 `protocol` 与 `command` 尚未连接到 stepper；开机不会初始化或操作电机 GPIO。

RMT 迁移阶段一已新增独立 `rmt_stepper_init()` 接口：它为 X STEP GPIO15 和 Y STEP GPIO8 创建两个 1 MHz RMT TX 通道，并建立同步管理器；初始化不调用 `rmt_transmit()`，不会产生 STEP 脉冲。RMT 阶段二新增 `rmt_stepper_execute()`，由 Planner 的单轴 block 调用；它使用 copy encoder 和 RMT symbol 输出脉冲。阶段三新增 `rmt_stepper_execute_xy()`，为两轴生成相同长度的主步时间轴并由同步管理器共同启动。阶段四将长 block 分为每块最多 64 个主步时间片，跨块保留 Bresenham 误差；全部常规单轴和 XY block 现已走 RMT，Feed Hold 在当前分块结束后暂停，Reset 停止活动 TX。

后续实现顺序：

1. `stepper_init()`：已实现，配置 RayCube X/Y 三组输出，并立即禁用两轴。
2. `stepper_set_enabled()`：已实现，按机型配置处理高有效禁用引脚。
3. `stepper_set_direction()`：已实现，写入方向；方向建立时间将在脉冲阶段的调用方处理。
4. `stepper_pulse()`：已实现，输出满足最小 3 us 宽度的单个 STEP 脉冲。

本模块不承担 mm 到步数的换算、加减速、插补、前瞻或队列管理；这些属于后续 `motion` 和 `planner` 模块。

## 单轴硬件打通计划

目标是验证 RayCube 的 X、Y 步进驱动电气链路可控，不是实现 G-code 运动。完成前，`protocol`、`command`、`motion` 和 `planner` 都不得调用本模块。

### 阶段一：安全初始化

在 `stepper_init()` 中遍历 `machine::raycube_axis()` 提供的两个轴配置：

1. 将 STEP、DIR、DISABLE 都配置为 GPIO 输出。
2. 将 STEP 写为其空闲电平，当前为低电平。
3. 将 DIR 写为固定初始方向。
4. 按 `disable_active_high` 将两个轴写为禁用状态。
5. 只有上述步骤全部成功后，初始化才返回 `ESP_OK`。

验收标准：刷写后设备静止；测量两个 DISABLE 引脚均处于禁用电平；STEP 输出保持低电平。

结束改动标准：不创建持续运行的 FreeRTOS 任务，不产生 STEP 脉冲；仅一次性硬件测试任务可初始化 stepper。

阶段状态：代码已完成，尚未接入 `app_main`，未进行构建或实机验收。

### 阶段二：使能和方向控制

实现 `stepper_set_enabled()` 与 `stepper_set_direction()`：

1. 根据 `disable_active_high` 将逻辑“使能”翻译为正确的 GPIO 电平。
2. 根据 `direction_inverted` 将逻辑方向翻译为正确的 DIR 电平。
3. 对非法轴编号返回 `ESP_ERR_INVALID_ARG`。
4. 只有 `stepper_init()` 成功后才允许写入引脚；否则返回 `ESP_ERR_INVALID_STATE`。

验收标准：手动调用使能/禁用后，驱动器保持力状态可观察；更改方向不会产生 STEP 脉冲。

结束改动标准：不将使能状态和运动位置写入 protocol 或 parser 状态。

阶段状态：代码已完成，尚未接入测试入口，未进行构建或实机验收。

### 阶段三：单脉冲输出

实现 `stepper_pulse()`：

1. 将指定轴的 STEP 写为有效电平。
2. 使用 ROM 延时保持至少 `kStepPulseMicroseconds`，RayCube 当前为 3 us。
3. 将 STEP 恢复为空闲电平。
4. 该函数只输出一个脉冲，不循环、不计算距离、不等待队列。

验收标准：逻辑分析仪或示波器观察到正确极性、宽度不小于 3 us 的单脉冲；在电机使能且低频调用时，电机只移动一个微步。

结束改动标准：不在普通 FreeRTOS 任务中用循环连续调用此函数，不宣称其具备实时运动能力。

阶段状态：代码已完成，使用 `esp_rom_delay_us()` 保持 STEP 高电平至少 3 us；尚未接入测试入口，未进行构建或实机验收。

### 阶段四：受控点动测试入口

在单独的测试入口中，以低频、固定次数调用阶段二和阶段三接口：

1. 先初始化，确认两轴禁用。
2. 一次仅使能 X 或 Y 轴。
3. 设置方向，等待方向建立时间。
4. 以保守间隔输出固定步数，例如 80 步。
5. 再次禁用该轴，并等待人工确认后才测试另一轴和反方向。

验收标准：X/Y 分别能正反运动；80 步约为 1 mm；上电、测试结束、异常路径均回到禁用状态。

结束改动标准：该测试入口仅用于实机校验，验收完成后不作为正式 protocol 功能保留。

阶段状态：实机验证已通过。临时测试程序及 `app_main` 调用已删除；当前启动路径不会操作电机。后续应由 `motion` 模块提供新的、明确受控的测试入口。

## 后续边界

单轴测试通过后才创建 `motion` 模块。`motion` 负责 mm 到整数步数和直线插补；`planner` 负责 block 缓冲、前瞻及速度曲线；stepper 仅执行已确定的 STEP/DIR 时序。
