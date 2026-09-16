# WebSocket 服务模块

`websocket` 是 HTTP 命令的异步观察通道，不是第二个 G-code 输入源。

## 当前架构

- 使用 ESP-IDF 原生 `esp_http_server` 的第二个实例，监听 TCP `8849`。
- 客户端通过 `ws://<设备IP>:8849/` 完成 HTTP Upgrade 后保持 WebSocket 连接。
- 最多保留 3 个 WebSocket 客户端；第 4 个连接会在握手后关闭。
- HTTP `8848` 的 `/command` 执行完成后，把同一份原始 Grbl 风格响应广播为 WS 文本帧。
- WebSocket 接收到的帧只会被读取并丢弃，不能提交运动或系统命令。
- 发送被投递到 `8849` 所属的 HTTPD 工作队列。工作队列满时丢弃最新通知，不阻塞
  protocol、service、planner 或运动执行路径。

## 分层

`HTTP /command → protocol → service → HTTP 响应 + WebSocket 广播`

WebSocket 不访问 planner、stepper、Timer/RMT 或激光控制。运动命令仍只能由 HTTP、UART
或未来统一输入适配层提交到 protocol。

## 验收

1. Wi-Fi 启动后日志依次包含 `httpd: started port=8848` 和
   `websocket: started port=8849 clients=3`。
2. 三个客户端可同时连接 `ws://<设备IP>:8849/`。
3. 对 `http://<设备IP>:8848/command?commandText=%24I` 发起请求时，HTTP 回复与三个
   WebSocket 客户端收到的文本完全相同。
4. 关闭任一 WS 客户端后，其他客户端仍可收到后续命令回复。
5. 第四个 WS 客户端不会长期占用连接槽位。
