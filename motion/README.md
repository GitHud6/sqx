# 运动请求模块

`motion` 是 G-code 与 planner 之间的提交边界。

## 当前职责

- 校验 `MotionRequest` 的 XY 目标、进给、方向、软限位和激光功率。
- 将单条请求或圆弧离散得到的一批请求提交给 planner。
- 只有 planner 接受请求后，调用方才能提交对应的解析器状态。

本模块不产生 STEP、不修改实际机床位置，也不直接访问 RMT、Timer 或 GPIO。
