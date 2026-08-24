# SD 文件与作业模块

## 目标分析

`sd` 模块负责把已挂载的 SD 卡文件转换为可供控制器执行的 G-code 行。
它位于 SD 硬件驱动和命令执行链之间，目标是支持当前普通 `.gc` 灰度雕刻文件，
并支持通过统一读取接口接入 gzip 压缩文件。

模块边界如下：

```text
driver/sdcard_driver
    -> sd 文件读取模块
    -> 完整 G-code 行
    -> execute_line()
    -> Planner
    -> Stepper / RMT / Laser
```

`driver/sdcard_driver` 只负责 SDMMC 挂载和卸载，不负责路径、文件格式或 G-code。
`sd` 模块也不直接操作 STEP、RMT 或激光 GPIO。

## 目录规划

```text
sd/
  README.md
  file_reader.hpp          // 统一文件读取抽象
  plain_file_reader.cpp    // 普通文本文件读取，第一阶段实现
  plain_file_reader.hpp
  sd_job_service.cpp       // 后续实现 SD 作业调度
  sd_job_service.hpp
```

## 读取抽象

文件读取接口应隐藏底层文件格式，使作业调度不依赖普通文件或压缩文件：

```cpp
class FileReader {
public:
    virtual bool open(const char* path) = 0;
    virtual bool read_line(char* buffer, std::size_t capacity) = 0;
    virtual bool eof() const = 0;
    virtual bool has_error() const = 0;
    virtual void close() = 0;
    virtual ~FileReader() = default;
};
```

第一版由 `PlainFileReader` 通过 `/sd` 挂载点读取普通 `.gc` 文件。
未来增加 `GzipFileReader` 后，`sd_job_service` 不需要改变，只替换文件读取器实现。

## G-code 行规则

`PlainFileReader` 负责文件层面的行读取，不负责 G-code 语义解析：

- 支持 `\n`、`\r` 和 `\r\n` 行结束符。
- 去除行结束符，但保留有效 payload。
- 空行可以直接跳过，或由作业层判定为无需执行。
- 超过固定缓冲区的单行必须报告读取错误，不能截断后执行。
- EOF 只能表示文件正常结束，底层读取失败必须单独报告错误。
- 注释和 G-code 单词仍交给 `execute_line()` 处理，避免 SD 输入和 UART 输入语义不一致。

## 作业调度边界

后续 `sd_job_service` 应采用单一命令执行者模型：

```text
SD 文件读取
    -> 命令行队列
    -> 唯一协议/命令执行上下文
    -> execute_line()
    -> Planner 背压
```

SD 任务不能与 UART 任务同时直接修改同一个 `ParserState`。当前最快的验证版本可以
暂时只启用 SD 输入，但最终应让 UART 和 SD 共用同一个命令提交入口。

Planner 满载时，SD 文件行不能丢失或重复执行，应等待空间释放后再提交。文件 EOF 后，
还必须等待 Planner、Segment 和 RMT 排空，再结束作业并关闭激光。

## gzip 文件读取

当前已通过 `GzipFileReader` 实现 gzip 文件的流式读取。它使用 ESP-IDF
提供的 `tinfl` 底层解压接口，不把整个压缩文件加载到内存：

```text
普通 .gc      -> PlainFileReader
压缩 .gz      -> GzipFileReader -> 解压后的 G-code 行
```

gzip 实现自行维护压缩输入缓冲、解压状态、CRC32 和原始长度校验，不能让压缩库对象直接泄漏到
`execute_line()`、Planner 或激光模块。

## 实现步骤与验收标准

### 阶段一：定义读取接口

实现内容：

1. 创建 `sd/` 模块目录和 `FileReader` 接口。
2. 明确打开、逐行读取、EOF、错误和关闭的生命周期。
3. 确定固定行缓冲容量和超长行处理策略。

验收标准：

- 普通文件和未来 gzip 文件可以共享同一读取接口。
- EOF、读取错误和超长行能够区分。
- 接口不依赖 Arduino `File`、G-code 解析器或 Planner 类型。
- 协议层已经预留 `$SD/STATUS`、`$SD/LIST` 和 `$SD/RUN=<path>` 的命令分发入口，
  但 `$SD/RUN` 此阶段只允许完成参数校验和状态登记。

结束改动标准：

- 读取接口已稳定，后续文件格式可以独立替换实现。

### 阶段二：实现普通 `.gc` 文件读取

实现内容：

1. 使用 `/sd` 挂载点打开普通文本文件。
2. 实现跨缓冲区的逐行读取。
3. 处理 CR/LF、空行、EOF、超长行和底层读取错误。
4. 增加文件读取统计，至少包括行号和读取错误状态。
5. 实现 `$SD/STATUS` 的真实挂载状态查询。
6. 实现 `$SD/LIST` 对 `/sd` 根目录的枚举，输出文件名、大小和类型。

验收标准：

- 能完整读取灰度 `.gc` 文件，不丢行、不重复、不截断。
- 文件最后一行没有换行符时仍能正确返回一次。
- 超长行不会被提交给 `execute_line()`。
- 读卡失败与正常 EOF 不会混淆。
- 串口发送 `$SD/STATUS` 能得到真实挂载状态；未挂载时返回 `error:sd-not-mounted`。
- SD 已挂载时，`$SD/LIST` 能列出 `/sd` 根目录文件、大小和类型，并且列表结束后只返回一次 `ok`。
- 目录枚举失败时返回 `error:sd-list-failed`，不返回成功响应。

结束改动标准：

- `PlainFileReader` 可以独立完成普通 `.gc` 文件读取测试。

### 阶段三：接入 SD 作业调度 

实现内容：

1. 增加固定容量的 SD 命令队列或与现有协议层共享统一入口。
2. 让 SD 行进入唯一的 `execute_line()` 执行上下文。
3. Planner 满载时形成背压，不丢失文件行。
4. 实现作业开始、运行、EOF 排空、完成、取消和错误状态。
5. 作业结束时关闭激光，并确认运动输出已经安全停止。
6. 通过 `$SD/RUN=<path>` 启动作业，通过 `$SD/STATUS` 查询 `starting`、`running`、
   `draining`、`completed` 或 `faulted` 状态。
7. `$SD/STATUS` 必须输出当前读取行号、作业故障类别和底层 G-code 错误，避免只返回
   `accepted` 后无法判断任务为什么停止。

验收标准：

- SD 文件中的每条有效 G-code 行最多执行一次。
- ParserState、S 值、运动顺序和最终坐标与文件一致。
- Planner 满载时不覆盖旧命令、不提前丢弃文件行。
- EOF 后不会立即关闭仍在执行的运动，能够正确完成排空。
- 取消、读取错误和复位路径都能关闭激光。
- `$SD/RUN=<path>` 能通过串口提交一个普通 `.gc` 文件，返回一次“已接受”响应，
  并能查询到作业处于待执行或运行状态；在本阶段结束前不把“已接受”当作“已雕刻完成”。
- `$SD/RUN=<path>` 启动后，`$SD/STATUS` 能观察到作业状态变化；文件读取失败或命令执行失败时进入
  `faulted`，正常 EOF 且 Planner 排空后进入 `completed`。
- 作业故障后，`$SD/STATUS` 能定位到失败行号，并区分 `open-failed`、`read-failed` 和
  `command-rejected`；后者还必须输出 `execute_line()` 的具体错误文本。

结束改动标准：

- 普通 `.gc` 灰度文件能够通过 SD 完整进入现有运动和激光链路。

### 阶段四：增加 gzip 文件读取器

实现内容：

1. 选定 gzip 解压库并封装在 `GzipFileReader` 内部。
2. 复用 `FileReader` 的逐行输出语义。
3. 区分压缩文件损坏、底层读卡失败和正常 EOF。
4. 保持普通 `.gc` 路径不受影响。
5. `$SD/RUN=<path>` 根据 `.gz` 后缀选择 `GzipFileReader`，其他路径保持使用
   `PlainFileReader`。

验收标准：

- `.gc.gz` 解压后的行序列与原始 `.gc` 文件一致。
- 解压错误不会被误报为正常作业完成。
- gzip 缓冲区不会阻塞 UART 实时控制和运动执行。
- 普通 `.gc` 和 `.gc.gz` 使用相同的作业调度接口。
- gzip header、deflate 数据、CRC32 或原始长度校验失败时，作业进入 `faulted/read-failed`，
  不允许误报为正常 EOF。

实机验收：

1. 使用 `$SD/LIST` 确认卡上的 `.gz` 文件名。
2. 发送 `$SD/RUN=/实际文件名.gz`，确认只返回一次 `accepted` 和 `ok`。
3. 轮询 `$SD/STATUS`，确认行号递增，且作业进入 `running` 或 `draining`。
4. 对包含当前已支持 G-code 的压缩测试文件，确认最终状态为 `completed`；如果文件包含尚未迁移的
   LightBurn 命令，应进入 `faulted|fault=command-rejected`，这仍能证明 gzip 已成功读到该行。

结束改动标准：

- 压缩文件支持成为 `FileReader` 的可替换实现，而不是修改命令解析和 Planner。

## SD 串口诊断与作业命令

SD 命令属于协议层系统命令，不进入 `execute_line()` 的 G-code 解析路径。
第一版定义以下命令：

```text
$SD/STATUS
$SD/LIST
$SD/RUN=/gray_test.gc
```

命令语义：

- `$SD/STATUS`：返回挂载状态、挂载点和可用的基础介质信息。
- `$SD/LIST`：列出 `/sd` 根目录下的文件名和大小，完成后只返回一次 `ok`。
- `$SD/RUN=<path>`：提交 SD 作业请求。第一阶段只要求命令入口、路径校验和任务状态接口可达，
  不要求立即执行文件内容。
- 文件列表或作业启动失败时返回明确的 `error:sd-...`，不能伪造 `ok`。
- 诊断信息和协议响应不能通过同一个 UART 输出通道交错打印。

阶段一中，`$SD/STATUS` 只报告当前挂载状态，`$SD/LIST` 返回
`error:sd-list-not-ready`，`$SD/RUN=<path>` 只返回路径已登记的保留响应；
这些响应不代表已经读取文件或完成雕刻。阶段三开始后，`$SD/RUN` 才会启动普通 `.gc` 作业。

推荐响应示例：

```text
[SD:mounted path=/sd|job=idle]
ok
```

```text
[FILE:/gray_test.gc|SIZE:182736]
[FILE:/test.gc|SIZE:4096]
ok
```

```text
[SD:run accepted path=/gray_test.gc]
ok
```

`$SD/RUN` 真正开始执行后，作业层仍必须通过唯一的 `execute_line()` 上下文提交每一行，
不能让 SD 任务和 UART 任务并发修改同一个 `ParserState`。

## 当前非目标

- 不在本模块实现 SD 卡硬件挂载；由 `driver/sdcard_driver` 负责。
- 不在阶段一实现 gzip 解压；当前 gzip 功能属于阶段四。
- 不实现 WebUI 文件管理、上传、删除和文件列表。
- 不复制 `ray2-main` 的 Arduino 全局对象和完整多客户端系统。
