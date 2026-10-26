# ESP32-C3 BLE 测试外设

这是 NETGATE 的独立 ESP-IDF 测试夹具工程，用于给 ESP32-S3 网关提供一个可控的 BLE 外设。它不是网关固件的一部分，也不会参与根项目构建。

## 设备变体

fixture 使用 `NETGATE_C3_FIXTURE_VARIANT` 编译宏切换整套 BLE 身份。默认值为 `0`，使用
`Netgate-C3-Device0`；设置为 `1` 后使用第二套独立的广播名和 UUID。两套变体的 GATT 行为相同，便于验证
网关通过 MQTT 注册不同设备。

| 宏值 | 广播名称 | Service UUID | RX UUID | TX UUID |
| --- | --- | --- | --- | --- |
| `0` | `Netgate-C3-Device0` | `b3a5f6c0-1f3d-4d91-8a50-53d0a4a07301` | `...7302` | `...7303` |
| `1` | `Netgate-C3-Device1` | `c4b6f7d1-2f4e-5e92-9b61-64e1b5b08412` | `...8413` | `...8414` |

可以直接修改 `main/main.c` 中的默认宏，也可以在 CMake 配置时传入：

```text
-DNETGATE_C3_FIXTURE_VARIANT=1
```

使用仓库脚本时可以直接指定变体：

```powershell
pwsh.exe -NoLogo -NoProfile -File .\tools\build_flash_c3_fixture.ps1 -Port COM9 -Variant 1
```

烧录变体 `1` 后，S3 的远程 Profile 需要把 `advertised_service_uuid` 和 `services[0].uuid` 设置为
`c4b6f7d1-2f4e-5e92-9b61-64e1b5b08412`，RX/TX 特征改为 `...8413` / `...8414`，并可将
`advertising_name` 写成 `Netgate-C3-Device1` 作为日志参考。源码中的 UUID 字节仍按 NimBLE 小端顺序填写。

## 行为

- 主广播包携带所选变体的 Service UUID；对应广播名称位于 Scan Response。
- 连接后，等待网关订阅 TX 特征。
- 每 2 秒经 TX 发送一条 JSON 遥测通知。
- 网关写入 RX 特征时，C3 串口打印收到的命令内容。
- 断开后恢复广播。

## GATT 协议

| 项目 | UUID | 属性 | 用途 |
| --- | --- | --- | --- |
| Service | 由 `NETGATE_C3_FIXTURE_VARIANT` 选择 | Primary Service | 与对应远程 Profile 配置一致 |
| RX | 由 `NETGATE_C3_FIXTURE_VARIANT` 选择 | Write, Write Without Response | 接收网关命令并打印到 C3 串口 |
| TX | 由 `NETGATE_C3_FIXTURE_VARIANT` 选择 | Notify | 向网关周期发送 JSON 遥测帧 |

源码中的 `BLE_UUID128_INIT` 使用 NimBLE 所需的小端字节序；表格显示的是标准 UUID 文本顺序。

两条特征都有标准 `0x2901 Characteristic User Description` 描述符。BLE 工具仍可能把自定义 UUID 显示为 `Unknown Characteristic`；读取该描述符即可获得文本含义。

网关作为 Central 会在连接后主动发起 ATT MTU Exchange，fixture 以 `256` 作为首选 MTU 响应。
协商成功后的单条 Notify 最大应用载荷为 `MTU - 3`，即最多 `253` 字节；超过该上限的私有协议仍需自行分包。

S3 网关默认最多同时维持两条外设连接。它会顺序发起连接请求：第一条连接建立后继续扫描，直到两条连接都就绪；
达到上限后停止扫描，任一设备断开后恢复扫描补位。

TX 帧示例：

```json
{"sequence":1,"temperature_c":26,"voltage_mv":3701}
```

## 使用方式

1. 在 VS Code 中单独打开 `test_devices/esp32c3_ble_fixture` 目录，或将它作为独立 ESP-IDF 项目导入。
2. 确认目标芯片为 `esp32c3`，串口选择 `COM9`。
3. 由操作者执行构建、烧录和监视；根项目约束不允许自动进行这些操作。
4. 运行 S3 网关。它应按 `advertised_service_uuid` 发现 C3，获取可用的 Scan Response 名称后连接、发现 GATT 服务、订阅 TX，然后把原始 JSON 帧交给 MQTT 链路。

## 验收观察点

- C3 串口出现所选变体的 `Advertising as ...` 日志。
- S3 串口出现 UUID 命中、名称、MAC、RSSI、连接、服务发现和通知订阅日志。
- C3 每两秒打印 `TX telemetry`。
- PC 订阅 MQTT 时可看到对应 Profile 的 telemetry 事件。
