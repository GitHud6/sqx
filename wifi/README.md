# Wi-Fi 模块

`wifi_manager.c/.h` 初始化 ESP-IDF STA、维护 Wi-Fi 凭据并将 IP 可用状态通知网络管理器。

- 初始凭据由 Kconfig 默认值提供；BLE 配网成功后写入 NVS `wifi_cfg` namespace。
- STA 获取 IP 后通知网络管理器启动 MQTT。
- 断开时停止 MQTT，并采用延迟/退避方式重新连接。
- 配网候选凭据连接成功后才提交为持久配置；日志不应输出密码。

该模块只负责 Wi-Fi 生命周期，不维护 MQTT Topic、BLE Profile 或 OTA 下载流程。
