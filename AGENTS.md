# 项目规则

- 未收到明确的提交命令时，不执行 Git 提交。
- 编写包含目标分析的 `.md` 文档时，必须同时给出实现步骤和各步骤的验收标准。
- 按 `.md` 规划完成一个步骤且已得到对应验收后，开始下一步骤前必须先提交该步骤的改动。
G-code/parser
  只负责解析几何和运动请求

planner
  只负责 block、junction、entry/exit/nominal 速度

profile
  只负责单个 block 内的速度-时间-位移曲线

stepper_prep
  只负责按固定时间片积分，并量化整数步数、周期和 AMASS

segment ring
  只负责传递已经完成的 segment

timer/RMT
  只负责消费 segment 并产生脉冲