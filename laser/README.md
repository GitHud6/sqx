# 激光模块

`laser` 将每个运动段携带的功率值应用到 RayCube 的 PWM/ENABLE 输出。

## 当前行为

- 激光功率随 `StepSegment` 的执行 ID 和段边界更新，不依赖 UART 命令抵达时刻。
- 启动运动时应用首段功率；暂停、作业结束、reset 或执行故障时关闭输出。
- 机型配置为 GPIO4 PWM、GPIO5 ENABLE，2 kHz，最大功率 1000。

激光模块不参与速度规划，也不决定 STEP 时序。
