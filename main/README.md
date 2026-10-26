# 主程序模块

`main.c` 负责启动编排，不承载 BLE、Wi-Fi、MQTT 或 OTA 的业务实现。

启动顺序：初始化 NVS；创建 OTA 命令任务；初始化事件服务；注册 Driver；加载并注册 Profile；初始化 BLE Bus 和配网服务；启动 BLE；启动远程配置管理；初始化网络管理并启动 Wi-Fi；关键服务启动成功后确认 pending OTA 镜像。

`Kconfig.projbuild` 集中定义 Wi-Fi、MQTT Topic 和 OTA 命令大小。`CMakeLists.txt` 组合各 C 模块，并嵌入 `config/devices.json`。
