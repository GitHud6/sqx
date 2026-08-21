# machine

本模块保存当前实验目标机型 RayCube 的静态硬件配置，是协议、运动规划和步进驱动之间的板级边界。

当前只定义 X/Y 轴的 `STEP`、`DIR`、`DISABLE` 引脚、有效电平、每毫米步数、最大速度和加速度。数据来源为祖父项目的 `longer_raycube_v0p1.h`，并与 `D:\RAY2-main` 的 RayCube 配置一致。

| Axis | STEP | DIR | DISABLE | steps/mm |
| --- | ---: | ---: | ---: | ---: |
| X | GPIO15 | GPIO7 | GPIO16 | 80 |
| Y | GPIO8 | GPIO17 | GPIO18 | 80 |

RayCube 工作行程为 X=`130 mm`、Y=`110 mm`，坐标原点按当前运动实验约定为 `0`；运动层会拒绝超出 `[0, max_travel]` 的目标。

`DISABLE` 为高有效。后续步进驱动初始化时必须先将两个轴写为禁用状态；在此之前不得发出 STEP 脉冲。

本模块不初始化 GPIO，不读取配置，不产生运动。硬件版本变更时，应新增独立机型配置而不是修改 RayCube 常量。
