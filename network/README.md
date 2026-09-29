# 网络协调模块

`network` 是本地网络的协调层，包含两个互不替代的职责：

- `network_command_adapter`：将 HTTP 与 WebSocket 已解码的命令文本有界地提交到共享
  `protocol` / `service` 路径。
- `network_lifecycle`：依据 Wi-Fi 链路变化，有序启停 HTTP 和 WebSocket 监听服务。

它不实现 HTTP 路由、WebSocket 帧解析、Wi-Fi 驱动、G-code 解析、planner 或步进脉冲。

## 命令适配

```text
HTTP / WebSocket
        |
        v
network_command_adapter
        |
        +--> realtime control --> service / status formatter
        |
        +--> normal lines --> protocol --> command --> motion --> planner
```

适配器在非实时核心的独立任务中消费有界队列。网络请求不能直接写入 planner 或调用
Timer/RMT。普通文本会被拆分为完整行，并通过 `command_dispatch_with_response()` 执行，
因此 UART、HTTP、WebSocket 和 SD 作业保持同一套普通命令门控和安全语义。

### 输入规则

| 输入类型 | 行为 |
| --- | --- |
| 单个实时控制输入 | 立即分流；不进入普通行批次。 |
| 多行作业文本 | 创建一个输入生命周期批次，批次结束后等待执行链排空。 |
| 单行系统控制请求 | 不创建网络输入批次，避免独立作业启动继承短暂网络输入生命周期。 |
| 非命令文本 | 仅接受一个受支持的单行控制请求。 |

队列、提交互斥锁和批次保留标志共同限制并发请求。队列满、已有批次或适配器未启动时，
调用方收到 `Busy`、`Invalid` 或 `Unavailable`，而不是等待网络任务无限阻塞。

硬复位或安全中止后，`network_command_adapter_abort_pending()` 通过 generation 失效和队列
复位丢弃尚未消费的网络输入。该函数不清空 planner；运动中止仍由 `service` 统一拥有。

## 传输生命周期

```text
Wi-Fi link event
        |
        v
network_lifecycle task
        |
        +--> stop WebSocket --> wait for stop --> stop HTTP
        |
        +--> start HTTP --> start WebSocket
```

Wi-Fi 事件回调只通知本模块任务。所有同步 socket 停止和创建动作都在普通任务上下文执行，
不在 ESP 事件循环中运行。

启动时，生命周期任务会立即重新检查当前 Wi-Fi 状态：SoftAP 已启用，或 STA 已获得 IP 时，
启动 HTTP 与 WebSocket 服务。STA 断开时，旧 TCP 会话可能绑定已失效的地址，因此处理顺序是：

1. 先停止 WebSocket 并等待其任务退出。
2. 停止 HTTP。
3. 在 AP+STA 模式下，若 AP 仍可用则先恢复本地服务。
4. 请求 STA 重新关联；纯 STA 模式等待新的 `STA_GOT_IP` 后再启动服务。

无线设置应用请求不在协议或 Wi-Fi 事件回调中直接重启 socket，而是经由
`network_lifecycle_request_wifi_restart()` 走同一条串行路径。

## 修改步骤

1. 新增网络客户端时，只调用 `network_command_adapter_submit()`；不得建立平行的运动或
   service 状态路径。
2. 修改队列深度、批次语义或实时字节前，验证 HTTP、WebSocket 与 UART 的命令门控一致。
3. 修改 Wi-Fi 重连或服务启停时，只改 `network_lifecycle`；保持 Wi-Fi 回调为非阻塞通知。
4. 修改后分别测试 AP、STA、AP+STA 下的首次连接、页面刷新、STA 断开和 DHCP 地址变化。

## 验收标准

- 网络普通命令与 UART 普通命令经过同一 `protocol/service` 门控。
- 实时控制不会被普通行组装或批次排空延迟。
- SD 作业运行时，网络普通命令与其他客户端一样被拒绝；实时控制仍可用。
- STA 地址失效后不会向旧 HTTP 或 WebSocket socket 继续发送；新链路可用后服务恢复。
- 慢速浏览器、满队列或重复请求仅导致受控拒绝，不阻塞 Timer/RMT 实时路径。
