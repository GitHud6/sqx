# BLE 配网模块

## 模块定位

`ble_provisioning_service.c/.h` 让 S3 网关以 BLE GATT Peripheral 提供本地 Wi-Fi 配网入口。它与 `ble/` 中扫描外部传感器的 Central 总线并行但用途不同：手机/调试器连接网关写凭据；网关 Central 连接目标外设。

配网服务接收并验证写入，调用 Wi-Fi Manager 应用候选凭据，并通过状态特征报告结果；凭据持久化和重连策略由 `wifi/` 负责。

## GATT 服务

服务 UUID 基值为 `a0d9162f-8440-e38e-6a49-3a689713647c`（源码 NimBLE UUID 字节按小端排列）；广播名为 `NETGATE-<MAC 后缀>`。

| 特征 | UUID 尾字节 | 属性 | 作用 |
| --- | --- | --- | --- |
| SSID | `7d` | Write | 写入网络名称。 |
| Password | `7e` | Write | 写入 WPA/WPA2 密码。 |
| Control | `7f` | Write | 写入 `START` 或 `CANCEL` 命令。 |
| Status | `80` | Read / Notify | 读取或订阅 JSON 状态。 |

完整 UUID 和状态内容以源码及 Characteristic User Description 为准。

## 关键 API

| API | 用途 |
| --- | --- |
| `ble_provisioning_service_init()` | 注册配网 GATT 服务和 Wi-Fi 状态回调。 |
| `ble_provisioning_service_start()` | 启动配网 Peripheral 广播。 |
| `ble_provisioning_service_set_wifi_state(connected)` | 由 Wi-Fi 生命周期通知服务更新状态。 |

接口见 [`ble_provisioning_service.h`](ble_provisioning_service.h)。

## 联动流程

```text
Phone/tool --GATT Write--> provisioning service
    SSID/password -> candidate credentials
    START         -> wifi_manager_apply_credentials
Wi-Fi success -> persist credentials -> Status Notify
Wi-Fi failure/cancel -> report status; previous committed credentials remain
```

候选凭据只有连接成功后才提交为持久配置；连接失败不覆盖上次有效配置。Wi-Fi 断线和重连由 `wifi_manager` 处理，网络管理器再据 IP 状态控制 MQTT。

## 安全限制

当前服务没有强制 BLE 链路加密、配对鉴权或应用层认证，只适合受控学习环境。不要将它描述为安全配网，也不要在日志或 MQTT 状态中输出密码。
