# ray2-idf

ESP-IDF 5.5.1 上的 RayCube ESP32-S3 固件实现。当前可通过 UART 和 SD 文件执行
二维 G-code 雕刻作业。

## 当前运行基线

- 量产运动路径使用梯形速度规划。
- G0/G1 以及 G2/G3（离散为受限弦长的 G1 请求）可进入同一 motion/planner 链路。
- planner 使用 32 个 `PlannerBlock`，stepper 使用 64 个 `StepSegment`。
- STEP 时基为 Timer Group 0 / Timer 0，20 MHz；STEP 脉冲由 RMT channel 0/1 输出。
- SD `.gc` 与 `.gc.gz` 作业可完整运行；已实机确认无 segment underflow 和坐标失效。

## 模块文档

| 模块 | 文档 | 当前职责 |
| --- | --- | --- |
| `wifi` | `wifi/README.md` | ESP-IDF Wi-Fi SoftAP network service |
| `command` | `command/README.md` | G-code 解析、模态状态与圆弧离散 |
| `motion` | `motion/README.md` | 请求校验与 planner 提交 |
| `planner` | `planner/README.md` | 前瞻、速度边界、block 生命周期与供料 |
| `stepper` | `stepper/README.md` | profile 转换、DDA、segment 执行与位置 |
| `driver` | `driver/README.md` | Timer Group、RMT、UART 和 SD 外设所有权 |
| `protocol` | `protocol/README.md` | 行帧、实时命令、响应与状态报告 |
| `sd` | `sd/README.md` | 文件读取、gzip 解压与作业生命周期 |
| `homing` | `homing/README.md` | `$H` 与限位触发下的系统运动 |
| `laser` | `laser/README.md` | 段级激光功率输出 |
| `machine` | `machine/README.md` | RayCube 引脚、行程和轴参数 |

## 已知限制

- S 曲线不是当前稳定运行基线，不应用于验收或雕刻作业。
- 当前梯形作业的供料、DDA 相位和速度边界已稳定；部分复杂图案仍存在与
  `ray2-main` 不一致的电机声学表现，根因未确认，因此暂不继续修改运动热路径。

## 构建

在 ESP-IDF 5.5.1 PowerShell 环境中执行：

```powershell
idf.py build
```
