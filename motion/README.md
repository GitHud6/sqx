# 运动请求模块

`motion` 是 G-code 解析层与 `planner` 之间的运动请求边界。它将已解析的直线运动语义转为
可规划的 `PlannerBlock`，但不直接发 STEP 脉冲、不操作 GPIO，也不等待物理运动完成。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `motion_request.hpp` | 定义 `MotionRequest`、`MotionExecutionPlan`、运动类型和激光模式。 |
| `motion_submit.hpp/.cpp` | 校验请求，换算 X/Y 步数和方向，创建 `PlannerBlock` 并提交给 planner。 |

## 调用链路

```text
完整 G-code 行
  -> command::execute_line()
  -> MotionRequest
  -> motion_submit()
  -> planner_submit()
  -> planner / stepper_prep / GPTimer / RMT
```

`Accepted` 只表示 block 已安全进入 planner；它不表示电机已经开始或完成运动。调用方只有在
`Accepted` 后才能提交对应的解析器坐标和模态状态。

## 数据契约

`MotionRequest` 包含：

- `Rapid` 或 `Linear` 运动模式；
- 起点与目标机床坐标，单位为 mm；
- 当前进给，单位为 mm/min；
- 当前行生效的 S 功率和主轴模式；
- `has_motion` 标记。

`MotionExecutionPlan` 是对请求的离散化结果：

- X/Y/Z 每轴的绝对步数；
- 每轴正负方向；
- 初始主步间隔。

最终的加减速、junction 速度、segment 周期和 AMASS 由下游模块决定，`motion` 不得在这里
复制这些策略。

## 校验与换算

`motion_submit()` 会执行以下工作：

1. 检查请求、坐标、F 和 S 均为有限且合法的数值。
2. 检查 X/Y 起点和目标均在 RayCube 幅面内，且进给不超过每轴限制。
3. 按 `machine::raycube_axis()` 的 `steps_per_mm` 将 XY 位移换算为整数步数和方向。
4. 拒绝 Z 位移；当前机型只支持 Cartesian XY。
5. 根据路径长度、进给和主轴步数计算初始 step interval。
6. 构造携带起止坐标、路径长度、单位方向、目标速度和轴约束加速度的 `PlannerBlock`。

`Rapid` 使用模块定义的快速移动速度，并受 X/Y 最大速度限制；`Linear` 使用请求中的 F 值。

没有实际 XY 步数的 G-code 行不应进入本模块创建空 block；这类模态或功率更新由 `command`
层处理。

## 返回语义

| 结果 | 含义 | 上游行为 |
| --- | --- | --- |
| `Accepted` | block 已进入 planner。 | 提交 ParserState，并回复 `ok`。 |
| `Busy` | planner block ring 已满。 | 不提交 ParserState；由协议层按背压策略处理。 |
| `Rejected` | 请求非法、越界、Z 位移、规划器未就绪或拒绝 block。 | 不提交 ParserState，并返回对应错误。 |

## 模块边界

上游依赖：

- `command`：负责词法、G-code 模态、工作坐标到机床坐标转换和 ParserState 事务。
- `machine`：提供幅面、步距、速度和加速度限制。

下游依赖：

- `planner`：唯一的 block 接收者。

`motion` 不拥有实际位置。已规划位置由 parser/planner 保持连续，实际执行位置仅由实际 STEP
脉冲路径累计；归零、Reset 或执行失败后的可信度由 `stepper`、`planner` 与 `homing` 协同维护。

本模块不得反向依赖 `protocol`、`sd`、`laser`、RMT、GPTimer 或 GPIO 驱动。

## 当前范围

- 已支持：RayCube 的 `G0/G1` Cartesian XY 运动请求。
- 未支持：Z 运动、圆弧 `G2/G3`、其他运动学模型。
- 机床坐标的实际原点由 `$H` 建立；`G54` 的工作坐标转换在 `command` 层完成。

## 验收重点

- `G1 X20 Y10 F600` 能形成正确的 X/Y 步数、方向和目标坐标。
- 幅面边界内的目标被接受，越界目标被拒绝且不污染后续解析器坐标。
- planner 满时返回 `Busy`，不丢失或覆盖未执行 block。
- 连续短线请求保持入队顺序，S 值和主轴模式与来源请求绑定。
