# 机型配置模块

`raycube_machine` 是当前 RayCube XY 机型的唯一硬件参数来源。

| 轴 | STEP | DIR | ENABLE | 步距 | 行程 |
| --- | --- | --- | --- | --- | --- |
| X | GPIO15 | GPIO7 | GPIO16 | 80 steps/mm | 130 mm |
| Y | GPIO8 | GPIO17 | GPIO18 | 80 steps/mm | 110 mm |

- 两轴方向输出均按配置反相；限位输入为 X GPIO42、Y GPIO2，低电平有效。
- 轴最大速度为 12000 mm/min，加速度为 500 mm/s2。
- STEP 有效脉冲宽度为 3 us；实际波形由 `driver/rmt_step_pulse_driver` 输出。
- 激光为 PWM GPIO4、ENABLE GPIO5，2 kHz，最大功率 1000。

修改行程、步距、方向或引脚时，只修改该模块，并重新验证归零与软限位。
