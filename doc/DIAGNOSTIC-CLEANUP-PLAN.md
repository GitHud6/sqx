# 诊断设施清理计划

> 状态：已完成。阶段一至阶段四已分别提交；阶段五完成文档与残留说明清理。

## 目标

以 `deeb71e`（`修正G0按方向计算快速移动速度`）为稳定基线，删除为排查
45° 共振而临时加入的诊断输出、调试命令和观测计数，降低 Timer/RMT 热路径
负担，避免诊断用原子计数影响运动执行。

本计划只清理观测设施，不修改已经验证的 G0、G1、SD、planner、profile、
Bresenham、AMASS、segment 或 Timer/RMT 运动语义。

## 保留边界

必须保留：

- 实际位置与位置可信状态；
- 限位、急停、故障和安全状态；
- 作业生命周期与必要的作业进度；
- 线程同步、队列所有权和 ISR/任务通信所需的原子变量；
- 发生异常时用于定位故障的必要错误日志。

计划删除：

- `$POS/STATUS` 中的工程诊断快照；
- `$PROFILE/SELFTEST` 及其离线参考实现；
- 仅用于诊断的 planner/profile/segment/producer/timer/RMT/UART/TX 结构；
- 仅用于统计的原子计数器及其采样、格式化代码；
- 仅用于上述诊断文档的过时计划和验收记录。

## 实施步骤

### 步骤一：清理协议诊断入口

处理范围：

- 删除 `$PROFILE/SELFTEST` 命令识别、执行和格式化；
- 将 `$POS/STATUS` 恢复为正式位置状态接口，保留必要的实际位置字段；
- 删除 `$POS/STATUS` 对 planner、segment、timer、RMT、UART、TX 和 arc 诊断
  API 的调用；
- 删除仅为诊断接口保留的 include、格式化缓冲区和协议注释；
- 对未实现的 `0x18`、`!`、`~`，删除其统计输出，但不误删后续实现所需的
  接收/分发基础设施。

验收标准：

- `$PROFILE/SELFTEST` 不再被识别为有效命令；
- `$POS/STATUS` 只输出正式位置/状态信息，不再输出工程快照；
- 普通 G-code、SD 作业和已有位置设置命令的应答路径不变；
- 协议源码无悬空诊断函数引用。

### 步骤二：清理 Timer/RMT 与 segment 热路径统计

处理范围：

- 删除 Timer ISR 中仅用于统计的 interval、lateness、hot-path、AMASS 转换、
  边界和核心计数；
- 删除 RMT invalid-trigger 等仅用于诊断的状态；
- 删除 segment ring 的 high-water、underflow 统计接口中仅用于报告的部分；
- 删除 producer refill、batch、gap、service-cycle 等观测计数；
- 保留真实 underflow/执行故障检测所需的控制状态和错误处理。

验收标准：

- Timer ISR 不再执行诊断专用 atomic RMW；
- RMT 每次触发的实际 reset/start 行为不变；
- segment 消费、补充、完成和故障处理逻辑不变；
- 编译器无未使用字段、函数、include 或链接符号错误。

### 步骤三：清理 planner/profile/stepper prep 诊断设施

处理范围：

- 删除 planner diagnostics 结构、lookahead 快照和 velocity-collapse 统计；
- 删除 stepper prep、trapezoid generator、Grbl reference prep 的 self-test
  和格式化代码；
- 删除 profile/segment 生成过程中仅用于观测的计数器；
- 保留 planner 的实际速度规划、block 生命周期和执行完成同步。

验收标准：

- planner 仍正确生成 entry/exit/nominal/junction 速度；
- profile 和 stepper prep 的步数、时间片、周期和 AMASS 计算逻辑无语义修改；
- 不再存在仅供调试的 self-test 源文件或公开接口；
- 代码依赖关系清晰，无诊断模块反向参与运动控制。

### 步骤四：清理通信与外围诊断统计

处理范围：

- 删除 realtime 命令计数、UART 接收统计和 TX 队列统计；
- 删除 arc 诊断快照中不属于实际几何执行所需的字段；
- 保留通信错误处理、队列同步、正常应答和必要的安全控制。

验收标准：

- 正常命令仍能接收、解析、执行并返回 `ok` 或错误码；
- 不再维护与业务无关的 realtime/UART/TX 原子计数；
- SD 作业期间的通信权限和阻塞规则不变。

### 步骤五：文档和残留清理

实施状态：已完成。

处理范围：

- 删除或标记过时的诊断专项计划、日志说明和 self-test 验收记录；
- 更新模块 README，明确运动核心已冻结、诊断设施已移除；
- 全局搜索已完成；保留的 `stepper_segment_diagnostics()` 仅提供执行器正常运行所需的
  segment 状态快照，不再包含纯统计字段。

验收标准：

- 文档不再把已删除的调试命令或快照字段描述为可用功能；
- 源码中无孤立诊断实现、无悬空头文件声明；
- 变更范围仅限诊断清理，不包含共振规避或运动算法修改。

## 总体验收

阶段一至阶段四的固件验收已由用户确认通过。后续若扩展业务功能，应以最新提交为基线，
不得重新引入这些诊断计数或快照。

每一步完成后先进行针对性源码检查；全部步骤完成后再由用户授权执行编译和
烧录测试。最终至少验证：

- `G0 X100 F6000`；
- `G0 X100 Y100 F6000`；
- `G1 X100 Y100 F5000/F6000/F7000`；
- 单轴 `G1`；
- 圆弧和 SD 文件作业；
- 位置状态、限位和故障路径。

本清理计划不承诺消除 F5500～F6000 附近的机械共振；它的目的只是让后续
量产基线不再承担无必要的诊断开销，并为后续业务功能扩展保持干净边界。
