# 网络管理模块

`network_manager.c/.h` 将 Wi-Fi IP 生命周期与 MQTT 客户端生命周期连接起来。

启动流程是先初始化 MQTT 客户端，再启动 Wi-Fi STA；Wi-Fi 获得 IP 时启动 MQTT，失去 IP 时停止 MQTT。MQTT 自身负责 Broker 连接重试。BLE 协议处理不依赖本模块，网络不可用时 BLE 总线仍可运行。
