# 运动请求模块

`motion` 是 G-code 与 planner 之间的提交边界。

## 当前职责

- 校验 `MotionRequest` 的 XY 目标、进给、方向、软限位和激光功率。
- 将绝对坐标量化为 `MotionGeometry`，只输出各轴步数和方向。
- 将单条请求或圆弧离散得到的一批请求提交给 planner。
- 只有 planner 接受请求后，调用方才能提交对应的解析器状态。

## 边界约束

`MotionGeometry` 是本模块的唯一运动派生结果。它只包含几何量化数据：

- 各轴绝对坐标差对应的整数步数；
- 各轴的正负方向。

本模块不计算步进频率、周期、AMASS、segment 数量或 profile 阶段，也不根据设备频率限制
修改运动速度。上述时序数据由 profile 和 stepper-prep 在已冻结的 planner block 上生成。

本模块不产生 STEP、不修改实际机床位置，也不直接访问 RMT、Timer 或 GPIO。
