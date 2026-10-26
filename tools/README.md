# 本地开发工具

## 目录用途

`tools/` 存放开发机辅助脚本，不是设备端固件模块。当前脚本管理 ESP32-C3 BLE fixture；S3 主项目由 VS Code ESP-IDF 插件选择目标、配置、构建和烧录。

## C3 构建/烧录脚本

[`build_flash_c3_fixture.ps1`](build_flash_c3_fixture.ps1) 为独立 `test_devices/esp32c3_ble_fixture` 工程激活 ESP-IDF 5.5.1 环境并执行构建/烧录流程。

| 参数 | 含义 |
| --- | --- |
| `-Port COMx` | C3 串口，例如 `COM9`。 |
| `-Variant 0` | 选择 Device0 广播身份。 |
| `-Variant 1` | 选择 Device1 广播身份。 |

```powershell
pwsh.exe -NoLogo -NoProfile -File .\tools\build_flash_c3_fixture.ps1 -Port COM9 -Variant 1
```

执行前确认 IDF/Python 环境可用、串口没有被占用，并核对目标板串口，避免烧录到错误设备。构建输出和本地 sdkconfig 应保持忽略状态。

## 职责边界

- 不构建/烧录 S3 网关。
- 不安装或运行 MQTT Broker，不提供 OTA HTTP 文件服务器。
- 不修改设备 Profile，也不替代 OTA Manager。
- 不在脚本中写入 Wi-Fi/Broker 个人凭据或密钥。

C3 广播和 GATT 行为见 [`../test_devices/esp32c3_ble_fixture/README.md`](../test_devices/esp32c3_ble_fixture/README.md)；端到端 MQTT/OTA 操作见根目录 [`../README.md`](../README.md)。
