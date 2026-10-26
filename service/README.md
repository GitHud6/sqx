# 网关服务模块

`gateway_service.c/.h` 在 FreeRTOS 中创建固定容量的事件队列，并作为 Protocol 层的 event sink。

BLE 回调只将统一事件复制进队列；独立服务任务再调用 MQTT 发布，避免在 NimBLE 回调上下文执行 JSON 编码和网络 I/O。队列已满时返回超时错误，不做无界分配或离线补发。
