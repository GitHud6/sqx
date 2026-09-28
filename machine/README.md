# 机器 Profile 模块

`machine` 是驱动层、运动层、归零与激光控制读取机器参数的唯一入口。它不包含产品型号、
网络名称或用户配置；板级资源和运动限制由构建期 profile 决定。

## 配置来源

源文件是 `config/machine_profile.json`。其字段分为三类：

| 区域 | 字段 | 作用 |
| --- | --- | --- |
| 顶层 | `step_pulse_microseconds` | STEP 脉冲有效宽度。 |
| `axes` | STEP、DIR、ENABLE、限位 GPIO 和极性 | 轴输出与归零输入的硬件拓扑。 |
| `axes` | `steps_per_mm`、最高速度、加速度、行程 | 坐标量化、软限位和运动限制。 |
| `laser` | PWM、ENABLE GPIO、极性、频率、最大功率 | 激光输出边界。 |

当前执行链固定为两个正交轴，JSON 的 `axes` 数组必须恰好包含两个元素，顺序分别对应
`Axis::X` 和 `Axis::Y`。这不是运行期可切换的运动学定义。

## 构建期生成

`cmake/generate_machine_profile.py` 在编译前读取并校验 JSON，生成构建目录内的
`machine_profile_generated.hpp`。生成文件不会提交到仓库。

校验规则包括：

- 所有输出与限位 GPIO 必须在 ESP32-S3 有效编号范围内，且互不重复。
- 引脚极性必须是布尔值。
- `steps_per_mm`、最高速度、加速度和行程必须是有限的正数。
- PWM 频率、最大功率与 STEP 脉冲宽度必须为正整数。
- profile 必须符合当前 schema 版本并含有完整双轴和激光定义。

`machine_profile.hpp` 只公开稳定的 `machine_axis()`、`machine_laser()` 与编译期脉冲
宽度常量。下游模块不得直接读取 JSON，也不得复制引脚或几何参数。

## 修改步骤

1. 修改 `config/machine_profile.json` 中与目标板和机械结构对应的字段。
2. 重新构建。生成器必须成功，构建日志不应出现 profile 校验错误。
3. 在禁用激光输出或采取等效安全措施的条件下，验证 X/Y 的方向、ENABLE 极性和 STEP
   脉冲。
4. 执行 `$H`，验证两个限位输入的触发极性、归零方向和拉离行为。
5. 验证软限位、短直线运动和激光 PWM 输出，再恢复正常作业测试。

## 验收标准

- 不存在其他模块硬编码的机器 GPIO、行程、steps/mm 或激光最大功率。
- 修改不需要改变 `protocol`、`planner`、`stepper` 或 Timer/RMT 的职责边界。
- 配置非法时构建失败；配置合法时产物仍使用编译期常量，运行期没有 JSON 解析、堆分配或
  网页热修改路径。
- 机器硬件变更后通过方向、归零、限位、暂停、复位和激光关闭测试。
