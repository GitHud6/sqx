# 运动规划模块

`planner` 将 `MotionRequest` 组织为可前瞻的 `PlannerBlock`，并持续向 `stepper_prep`
供给已冻结的运动 block。它不操作 STEP/DIR/RMT。

## 当前实现

- 32 槽 block ring；已发布为 segment 的连续前缀冻结，后续 block 可继续参与前瞻。
- 使用 junction deviation、反向 pass 和正向 pass 求解相邻 block 的 entry/exit 速度边界。
- 梯形 profile 是当前稳定路径；SD 雕刻实测中，block 供给和 segment ring 均保持连续。
- 完成事件、feed hold、resume、reset 和执行故障由 planner 统一回收和清理。

## 运行诊断

每秒 `TRAPEZOID-TRACE` 输出 block/segment 供给、DDA 相位延续、junction 速度差、
计划周期和 underflow。统计仅在任务或段边界收集，不在每个 STEP ISR 内测时。

## 已知限制

S 曲线不是当前验收基线。复杂图案的剩余声学差异未归因到 planner，暂不改变其算法。
