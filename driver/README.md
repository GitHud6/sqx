# 硬件驱动模块

`driver` 独占 ESP32-S3 外设配置；上层只通过公开接口请求运动、激光、UART、SD 和限位行为。

## STEP 外设

| 资源 | 当前配置 | 所有者 |
| --- | --- | --- |
| Timer Group 0 / Timer 0 | XTAL 40 MHz，经 divider 2 得到 20 MHz，auto-reload | `step_timer_driver` |
| RMT channel 0 | GPIO15，4 MHz，0.25 us idle 加 3 us STEP 高电平 | `rmt_step_pulse_driver` |
| RMT channel 1 | GPIO8，4 MHz，0.25 us idle 加 3 us STEP 高电平 | `rmt_step_pulse_driver` |

Timer ISR 只执行预计算的 DDA 与段切换。RMT 触发路径只复位读指针并启动 TX；不在每个
脉冲读取状态寄存器或进行原子诊断计数。

## 其他驱动

- `uart0_driver`：普通命令和实时字节的接收边界。
- `sdcard_driver`：SD 卡挂载和底层文件访问。
- `limit_switch_driver`：限位输入与 ISR 通知。
- `laser_driver`：PWM/ENABLE 输出。

所有 STEP GPIO 只可由此模块的 RMT 路径驱动，不允许保留第二套输出后端。
