# BLE 配网模块

该模块将 ESP32-S3 作为 BLE GATT Peripheral，接收 Wi-Fi SSID 和密码并发起 STA 连接。

| 特征 | UUID 尾字节 | 属性 | 用途 |
| --- | --- | --- | --- |
| SSID | `7d` | Write | 写入网络名 |
| Password | `7e` | Write | 写入 WPA 密码 |
| Control | `7f` | Write | 写入 `START` 或 `CANCEL` |
| Status | `80` | Read / Notify | 查询及接收 JSON 状态 |

服务 UUID 的基值为 `a0d9162f-8440-e38e-6a49-3a689713647c`（NimBLE 源码按小端存储）。设备广播名为 `NETGATE-<MAC 后缀>`。

SSID 和密码通过 BLE 写入；当前服务不强制加密连接或配对鉴权，只适合受控学习环境。不得将其描述为安全配网实现。
