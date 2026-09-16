# HTTP 服务模块

`httpd` 是网络承载之上的最小只读 HTTP 服务。它不解析 G-code，不访问
`service`、planner、stepper、Timer/RMT 或激光控制。

## 当前接口

- `GET /` 返回服务标识文本。
- `GET /health` 返回 `{"service":"ray2-idf","status":"ok"}`。

服务仅在 Wi-Fi 已启动后创建。`$WIFI/BEGIN` 成功后也会确认服务已经启动，因而
首次 Wi-Fi 初始化失败、随后通过该命令恢复时仍可建立 HTTP 服务。

## 后续扩展边界

未来 HTTP、WebSocket 或网络 G-code 输入必须经过独立的认证/会话和输入源适配层，
再进入 `protocol → service`。HTTP handler 不得直接调用运动模块。

## 本阶段验收

1. 连接 SoftAP 或已关联 STA 后，访问 `http://<设备IP>/` 返回 HTTP 200 和
   `Ray2-IDF HTTP service`。
2. 访问 `http://<设备IP>/health` 返回 HTTP 200、`application/json` 和固定健康响应。
3. Wi-Fi 不可用时固件仍正常启动；UART、SD 作业和运动路径不依赖 HTTP 服务。
