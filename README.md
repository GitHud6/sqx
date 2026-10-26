# NETGATE

NETGATE 是一个面向学习与作品展示的 ESP32-S3 BLE 到 MQTT 网关原型。它重点展示如何用 C 和 ESP-IDF 将 BLE 总线、设备驱动、协议处理、网络传输与配置管理拆分为独立模块。

当前项目已在 ESP32-S3 网关和两台 ESP32-C3 测试外设上验证基本链路：S3 根据远程 JSON Profile 按广播 Service UUID 发现并连接外设，订阅 GATT Notify，将数据转换为统一事件后发布到 MQTT。设备名称仅用于识别日志，不作为扫描匹配条件。

## 架构

```text
MQTT JSON 配置
      |
      v
配置管理与 NVS <----> Driver Registry
      |                       |
      v                       v
BLE Bus -> GATT Driver -> Device Protocol
                              |
                              v
                    Gateway Event Queue
                              |
                              v
                       MQTT Publisher
```

- `ble/`：BLE 总线抽象及设备运行时管理。
- `driver/`：GATT 操作与设备驱动接口、驱动注册表。
- `protocol/`：通用数据解码和设备私有帧处理。
- `config/`：JSON Profile 校验、远程配置事务及 NVS 持久化。
- `provisioning/`、`wifi/`、`mqtt/`：BLE 配网、Wi-Fi 管理和 MQTT 通信。
- `ota/`：网关自身的 HTTP 固件升级、非活动 OTA 分区写入和启动确认。
- `service/`：将协议事件从 BLE 回调上下文隔离后交由 MQTT 发布。
- `test_devices/esp32c3_ble_fixture/`：独立的 C3 BLE 外设测试工程，不属于 S3 固件构建。
- `tools/`：C3 测试夹具的本地构建/烧录辅助脚本。

模块边界如上；完整 schema 由 `config/devices.json` 展示，BLE 总线、驱动和配置管理接口分别位于 `ble/`、`driver/`、`config/`。

## 已实现能力

- ESP32-S3 上运行 ESP-IDF 5.5.1 和 NimBLE Central。
- BLE 总线按广播包内的 Service UUID 匹配目标，连接后发现 GATT 服务与特征并订阅 Notify。
- JSON 描述设备、服务、特征、属性、通用驱动回调、解码类型和 MQTT Topic；普通字段可通过声明式解码配置接入。
- 固定设备槽位支持多个 Profile；当前固件已验证两台 C3 外设同时连接。
- MQTT 可远程接收完整 BLE 配置，验证后热加载并将有效配置保存到 NVS，失败时尝试恢复旧配置。
- MQTT OTA 命令可触发网关 HTTP 固件下载，写入非活动 OTA 分区，通过镜像校验后切换启动分区；新固件完成核心模块初始化后确认有效。
- Wi-Fi 支持 BLE 配网、保存凭据与自动重连；MQTT 随 Wi-Fi IP 状态启动和停止，并在连接异常后自动重连。
- BLE 事件通过固定容量队列与网络发布隔离。业务遥测目前使用 QoS 0；配置结果使用 QoS 1。

当前声明式解码器适合 `raw`、整数、字节序、缩放等简单格式。厂商私有帧、分包、CRC 或加密协议仍需要在 `protocol/` 增加 C 代码处理。

## 环境与构建

- ESP-IDF v5.5.1。VS Code ESP-IDF 扩展使用项目中的 `.vscode/` 配置。
- 网关目标为 ESP32-S3，Flash 配置为 16 MB，使用 `partitions.csv` 中的双 OTA 应用分区布局。
- C3 测试外设是独立 ESP-IDF 项目，构建与烧录请从 `test_devices/esp32c3_ble_fixture/` 启动。

网关的 Wi-Fi SSID、密码和 MQTT Broker URI 由 Kconfig 提供。Kconfig 中的 `changlang` 是开发测试默认凭据，不应作为个人或部署网络的凭据；首次使用前请在本地项目配置中替换 Wi-Fi 与 Broker 参数。不要把个人密码或私钥提交到公开仓库。MQTT URI 当前使用 `mqtt://`，TLS 尚未实现。

仓库约束暂不允许自动构建或烧录；应由开发者在 VS Code ESP-IDF 扩展中选择 ESP32-S3 目标后手动执行，并通过串口监视器观察日志。

### C3 测试外设

测试固件支持两种广播身份：

| 变体 | 广播名称 | 广播 Service UUID |
| --- | --- | --- |
| `0` | `Netgate-C3-Device0` | `b3a5f6c0-1f3d-4d91-8a50-53d0a4a07301` |
| `1` | `Netgate-C3-Device1` | `c4b6f7d1-2f4e-5e92-9b61-64e1b5b08412` |

例如在 PowerShell 中烧录第二种变体：

```powershell
pwsh.exe -NoLogo -NoProfile -File .\tools\build_flash_c3_fixture.ps1 -Port COM9 -Variant 1
```

第一种变体将 `-Variant 1` 改为 `-Variant 0`，并按实际串口替换端口。测试外设每两秒发送一帧 JSON 遥测数据；其特征为 RX Write / Write Without Response 和 TX Notify。测试工程位于 `test_devices/esp32c3_ble_fixture/`。

## 端到端验证

1. 在 S3 使用 BLE 配网连接本地 Wi-Fi，并确认串口出现 Wi-Fi 获取 IP、MQTT 连接成功的日志。
2. 分别烧录并启动 C3 Variant 0 和 Variant 1，确认它们报告各自的广播 Service UUID。
3. PC 订阅遥测 Topic：

   ```powershell
   mosquitto_sub -h <broker-ip> -p 1883 -t 'netgate/devices/+/telemetry' -v
   ```

4. 确认 S3 日志显示两台设备连接和 Notify 订阅成功，并在 PC 收到两个设备 ID 对应的遥测消息。
5. 如需验证远程注册，可在一个 PC 终端先订阅配置结果 Topic，并在另一个终端发布完整配置。测试期间确保 Broker ACL 允许该客户端访问对应 Topic：

   ```powershell
   mosquitto_sub -h <broker-ip> -p 1883 -t 'netgate/config/ble/result' -v
   ```

   在另一个终端执行：

   ```powershell
   mosquitto_pub -h <broker-ip> -p 1883 -t 'netgate/config/ble' -q 1 -f .\config\devices.json
   ```

6. 观察配置结果中的 `accepted`、`error` 和 `revision`，并确认 BLE 设备按最新 Profile 恢复连接。

默认事件 Topic 形式为 `netgate/devices/<device_id>/<topic>`，远程配置结果默认发布到 `netgate/config/ble/result`。配置字段包括 `schema_version`、设备广播 Service UUID、Driver、GATT Services/Characteristics、属性、回调集合、事件类型、Topic 和可选解码器；示例见 `config/devices.json`。

### 网关 OTA 更新

OTA 命令使用 MQTT Topic `netgate/ota`，消息为包含固件 HTTP 地址的 JSON：

```json
{"url":"http://<pc-ip>:8000/netgate_hello_world.bin"}
```

固件文件必须来自与当前 S3 固件一致的 ESP-IDF 构建产物。可在局域网 PC 上用 Python HTTP Server 提供文件：

```powershell
python -m http.server 8000 --directory <firmware-directory>
```

先在一个终端订阅状态，再从另一个终端发送命令：

```powershell
mosquitto_sub -h <broker-ip> -p 1883 -t 'netgate/ota/status' -v
mosquitto_pub -h <broker-ip> -p 1883 -t 'netgate/ota' -q 1 -m '{"url":"http://<pc-ip>:8000/netgate_hello_world.bin"}'
```

状态包含 `queued`、`downloading`、`failed` 或 `rebooting`，并在下载期间上报字节进度。实现只接受 HTTP、HTTP 200 和已知且不超过非活动分区容量的 Content-Length；ESP-IDF 会在结束 OTA 写入时校验镜像格式。设备重启后，核心服务初始化成功才将新镜像标记为有效，否则保留 bootloader 回滚机会。

## 已知限制

- MQTT 使用明文 TCP (`mqtt://`)；当前 Broker 链路没有 TLS 证书校验，也没有完整的 Broker 身份认证与 Topic ACL 部署方案。
- BLE 配网中的 Wi-Fi 凭据目前没有强制链路加密或配网鉴权。此实现仅用于受控学习环境，不应作为安全配网产品部署。
- MQTT 遥测使用 QoS 0，断网期间产生的事件可能丢失；重连后继续发送新事件，不提供离线缓存或补发。
- 初始化 NVS 遇到空间不足或版本不兼容时，当前启动代码会擦除默认 NVS 分区并重新初始化，可能清除已保存的 Wi-Fi 和 BLE 配置。
- OTA 数据通过明文 HTTP 下载，未做镜像签名或来源认证；仅适合隔离的学习网络。不要从不可信网络触发升级。
- OTA 代码已实现，但本工作流受项目约束未执行编译、烧录或设备升级验证；需要操作者手动验证升级成功与未确认镜像回滚。
- 当前项目验证目标是功能链路和模块化架构，不代表工业级可靠性或量产安全认证；不包含长期压力测试结果。

## 项目边界

NETGATE 用于学习和展示 ESP-IDF、C、BLE GATT、多设备接入抽象、JSON 配置、NVS 和 MQTT 的端到端实现。它不是已完成安全加固、OTA 闭环或量产验证的工业网关。
