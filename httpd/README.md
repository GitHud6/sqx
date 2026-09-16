# HTTP 服务模块

`httpd` 是网络承载之上的 HTTP 适配层。它不直接访问 planner、stepper、Timer/RMT
或激光控制；命令必须经 `protocol → service` 进入既有运动与安全路径。

## 当前接口

- `GET /` 返回空的兼容响应。
- `GET /command?commandText=<url-encoded-command>` 或
  `GET /command?plain=<url-encoded-command>` 接受单条或多条换行分隔命令，返回原始
  Grbl 风格响应。
- `GET /files` 返回 SD 根目录 JSON 列表。SD 未挂载返回 `503`，SD 作业活动时返回
  `409`，避免与雕刻流并发访问文件系统。

服务仅在 Wi-Fi 已启动后创建。`$WIFI/BEGIN` 成功后也会确认服务已经启动，因而
首次 Wi-Fi 初始化失败、随后通过该命令恢复时仍可建立 HTTP 服务。

`/command` 对 `?`、`0x18`/`18`、`!` 和 `~` 保留实时命令语义；其余命令通过
`command_dispatch` 的调用方私有响应捕获接口执行。HTTP 命令不会占用或污染 UART
回复队列。

接口端口固定为 `8848`，以兼容 branch-net 的网络入口。`/favicon.ico` 与 `/healthz`
刻意不注册：前者只是浏览器兼容噪声，后者只服务于 branch-net 专属 HTTP 看门狗。

## 后续扩展边界

branch-net 的业务路由包含 `/command`、`/files`、`/upload` 和 `/updatefw`。本项目
先迁移当前架构可安全承接的读取与命令入口：

| branch-net 路由 | 本项目状态 | 原因 |
| --- | --- | --- |
| `/command` | 已实现 | 使用 branch-net 兼容的 GET 查询参数入口，统一经 protocol/service 执行 |
| `/files` | 已实现（只读） | 使用现有 SD 挂载与作业状态 |
| `/upload` | 未实现 | 尚无 SD 写入事务、路径授权和热插拔互斥 |
| `/updatefw` | 不适用 | 当前为单 factory 分区，明确不提供 OTA |
| WebSocket | 未实现 | 需要独立的会话、订阅和输出所有权设计 |

后续 HTTP、WebSocket 或网络 G-code 输入都必须保留此分层。HTTP handler 不得直接
调用运动模块，也不得绕过 service 的复位、暂停、恢复和作业生命周期门控。

## 本阶段验收

1. 连接 SoftAP 或已关联 STA 后，访问 `http://<设备IP>:8848/` 返回 HTTP 200 空响应。
2. `GET /command?commandText=%24I`、普通 G-code 和 `?` 分别返回与 UART 一致的协议响应。
3. `GET /command` 的 `!`、`~`、`0x18` 仍分别经过 service 的暂停、恢复和硬复位路径。
4. SD 空闲时 `GET /files` 返回合法 JSON；未挂载或作业中按约定返回 `503` 或 `409`。
5. `/favicon.ico` 与 `/healthz` 返回 404；两者不属于当前产品 API。
6. Wi-Fi 不可用时固件仍正常启动；UART、SD 作业和运动路径不依赖 HTTP 服务。
