# 本地开发工具

`build_flash_c3_fixture.ps1` 用于独立构建和烧录 ESP32-C3 BLE 测试外设，可通过 `-Variant 0/1` 选择广播身份。

脚本从 ESP-IDF 5.5.1 环境调用 C3 fixture 工程；使用前检查本地 ESP-IDF 安装、Python 环境和串口参数。根项目的 S3 构建继续由 VS Code ESP-IDF 扩展管理，遵守 `AGENTS.md` 中不自动构建/烧录的要求。

网关 OTA 使用 `ota/` 固件模块，不由本目录脚本执行。OTA HTTP 文件服务器和 MQTT 命令示例见根目录 `README.md`。
