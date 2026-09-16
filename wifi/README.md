# Wi-Fi 模块

`wifi` 负责 ESP-IDF 网络栈和基础无线接口初始化，为后续 Telnet、HTTP 和 WebSocket 提供网络承载。它不解析 G-code，不直接调用 planner、stepper、Timer/RMT 或激光模块。

## 当前实现

- 使用 ESP-IDF 5.5.1 的 `esp_netif`、`esp_event` 和 `esp_wifi`。
- `app_main` 会在 Wi-Fi 前初始化默认 NVS 分区；只有 NVS 空页耗尽或版本不兼容时才擦除并重建。
- 使用 NVS 命名空间 `ray2_wifi` 保存三类配网数据：`sta_ssid`、`sta_password` 和 `radio_mode`。
- `radio_mode` 表示用户保存的 STA 连接策略（`AP` / `STA` / `AP_STA`），不记录瞬时连接结果；实际 AP/STA 链路状态由 `wifi_service` 在运行时维护。
- 物理无线模式固定为 `WIFI_MODE_APSTA`。SoftAP 始终启动，SSID 为 `LongerLaser_XXXX`，后缀取 AP MAC 的最后两个字节。
- 当 `radio_mode=AP` 时不调用 STA 连接；当 `radio_mode=STA` 或 `AP_STA` 时，使用 NVS 中的 SSID/密码发起 STA 连接。
- SoftAP 密码为 `12345678`，地址固定为 `192.168.0.1/24`，最多 3 个客户端。
- 事件回调只更新 Wi-Fi 状态并输出必要日志，不执行文件操作或运动控制。
- 当前已实现 AP、STA 和 AP+STA 的基础无线承载，以及 `httpd` 的 HTTP 命令入口和
  独立 `websocket` 的命令结果观察通道。Telnet、WebSocket 命令输入和 MQTT 仍未接入。

## 与现有架构的关系

```text
wifi（网络承载和连接状态）
        ↓
httpd:8848（HTTP 命令输入） → protocol → service → planner → stepper
        ↓
websocket:8849（命令结果广播，仅观察）
```

Wi-Fi 任务默认固定到 non-realtime core 1，避免与当前 core 0 上的 Timer/RMT 实时运动路径争用 CPU。Wi-Fi 连接丢失只影响网络客户端，不应直接改变已提交的运动；网络输入生命周期由 `service` 统一处理。

## 后续验收

1. 任意有效配置上电日志都出现 SoftAP SSID 和 `192.168.0.1`。
2. 手机或电脑能连接该 SSID，并获得 `192.168.0.x` 地址。
3. 网络栈启动失败时，UART/SD 雕刻仍能独立工作。
4. 后续接入 Telnet 后，网络 G-code 与 UART 共用 protocol/service 控制和安全语义。

## 串口设置协议

本阶段兼容 `ray2-main` 的三个持久化设置路径，命令和值均不区分大小写：

- `$STA/SSID` 查询已保存 SSID；`$STA/SSID=<ssid>` 校验可打印 ASCII 字符和长度后保存 SSID。
- `$STA/PASSWORD` 仅返回是否已设置及长度；`$STA/PASSWORD=<password>` 校验可打印 ASCII 字符和长度后保存密码，绝不回显密码明文。
- `$RADIO/MODE` 查询目标模式；`$RADIO/MODE=AP|STA|AP_STA` 保存目标模式。
- `$WIFI/BEGIN` 不接受参数。它先停止当前 Wi-Fi，再读取 NVS 并按已保存模式重启无线模块。

设置命令只更新 NVS，不会停止或重启当前 Wi-Fi；仅 `$WIFI/BEGIN` 应用已保存的连接策略，并确认 HTTP 与 WebSocket 服务已经启动。实际射频始终是 APSTA；`AP` 只关闭 STA 关联动作，`STA` 和 `AP_STA` 都会尝试连接外部路由器。STA 未配置 SSID 时仍保留 AP 并输出警告。`$RADIO/STATE`、蓝牙、Telnet、WebSocket 命令输入和 MQTT 均不属于本阶段。

## `$I` 设备信息

`$I` 使用 Grbl 风格逐行返回固件、板卡、机型、软件版本、激光波长/功率和当前工作尺寸。Wi-Fi 已启动时，它追加运行时网络信息：

- STA：`[MSG:Mode=STA:SSID=...:Status=Connected|Connecting:IP=...:MAC=...]`
- AP：`[MSG:Mode=AP:SSDI=LongerLaser_XXXX:IP=...:MAC=...]`。`SSDI` 是为兼容量产协议保留的历史字段拼写；`XXXX` 为 AP MAC 的最后两字节。

实际射频始终是 APSTA；当连接策略为 `STA` 或 `AP_STA` 时，若 STA 已启用则同时输出 STA 行和 AP 行。连接策略为 `AP` 时只输出 AP 行。STA 仅在收到 DHCP 地址后报告 `Connected`；在关联中、断链或尚未获得地址时报告 `Connecting`，且 IP 固定输出 `0.0.0.0`。IP 与 MAC 来自实际 ESP-IDF 接口；Wi-Fi 未启动时输出 `[MSG:Mode=OFF]`。

## 本阶段验收

1. 三个查询命令均返回对应持久化值，密码不以明文出现。
2. 三个写入命令返回 `ok`，断电重启后查询值不变。
3. 非法 SSID、超长密码或未知模式返回错误，原有值不被覆盖。
4. 设置命令执行期间不改变 AP、planner、stepper 或现有雕刻任务状态。
5. `$WIFI/BEGIN` 后，`AP` 能重新建 AP，`STA` 能发起入网，`AP_STA` 在 STA 入网失败时仍保留 AP。
