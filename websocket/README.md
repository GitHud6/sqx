# WebSocket 模块

`websocket` 为本地网页门户提供一个独立的实时双向会话。它使用原始 TCP socket 和
`select` 实现 WebSocket 握手及帧收发，不依赖 ESP-IDF HTTPD 的 WebSocket upgrade 功能。
HTTP 服务与 WebSocket 服务拥有不同的监听器和任务生命周期。

该模块不解析 G-code，不直接调用 `service`、`planner`、`stepper`、Timer/RMT 或激光输出。
收到的命令文本只能经 `network_command_adapter` 进入共享协议路径。

## 会话模型

服务只保留一个活跃门户客户端。新连接到来时，会先关闭旧连接，再分配递增的 session
编号。待发送消息可绑定某个 session；发送前会丢弃不属于当前 session 的消息，防止页面
刷新或网络切换后向已关闭 fd 写入数据。

```text
browser
   |
   v
WebSocket listener --> one active session --> network_command_adapter
   ^                         |
   |                         v
status / response <--- bounded outbound queue
```

握手成功后，服务为新会话推送当前状态快照和构建信息。客户端主动关闭、协议错误、网络
收发错误或新客户端替换都会结束当前会话；监听器继续等待下一次连接。

## 帧与资源边界

服务运行在非实时核心的专用任务中，socket 保持非阻塞，循环通过短超时 `select` 同时等待
监听器、客户端可读事件和可写事件。

| 项目 | 行为 |
| --- | --- |
| 入站帧 | 支持完整、掩码的文本或二进制帧，以及 ping、pong、close 控制帧。 |
| 非法帧 | 未完成分片、超出固定容量、无掩码或不支持的长度编码会关闭当前会话。 |
| 出站消息 | 使用固定消息池和有界队列；发布调用不等待 socket I/O。 |
| 实时状态 | 只保留尚未发送的最新状态快照，旧快照可被替换。 |
| 发送受阻 | 遇到 `EAGAIN` 或 `EWOULDBLOCK` 时保留进度，等待下一次可写事件继续发送。 |

握手、接收和命令暂存缓冲属于 WebSocket 任务的静态存储，不占用任务栈。队列和消息池在
服务启动时创建，数量固定；慢速浏览器只能耗尽自己的有界通知预算，不能阻塞协议或运动
热路径。

## 生命周期

`network_lifecycle` 是服务启停的唯一协调者。Wi-Fi 链路可用时它启动监听器；STA 地址失效
或无线设置应用时，它先请求 WebSocket 任务退出，再等待任务完全停止，随后才停止或重建
HTTP 服务。该顺序避免新连接复用旧网络接口的 fd 状态。

`websocket_service_stop()` 只发出停止请求；调用方必须通过
`websocket_service_is_started()` 确认任务已经退出后，再释放或重建相邻网络服务。

## 修改步骤

1. 变更帧解析时，保持非阻塞读写、掩码校验和固定容量限制；不要在 socket 任务中等待
   文件系统、planner 或外部网络操作。
2. 变更会话替换或发送队列时，保持 session 编号校验，确保旧会话的待发消息不会投递给
   新客户端。
3. 变更入站命令处理时，只通过 `network_command_adapter` 提交普通输入和实时控制。
4. 变更启动或停止行为时，同时检查 `network_lifecycle` 的等待逻辑与 Wi-Fi 重连路径。

## 验收标准

- 页面刷新或重新连接后，只有最新客户端保持活跃会话。
- 刷新、断连和 DHCP 地址变化不会引起向旧 fd 的发送或访问已释放会话缓冲。
- 慢速浏览器或满出站队列不会阻塞 `protocol`、`service` 或 Timer/RMT 路径。
- ping/pong、关闭帧、分段 TCP 接收和非阻塞部分发送均能保持会话状态一致。
- 断开 Wi-Fi 或应用无线设置后，监听任务可被有界时间内停止，并能在新链路上重建。
