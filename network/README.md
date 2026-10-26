# 网络管理模块

## 模块定位

`network_manager.c/.h` 是 Wi-Fi 与 MQTT 的生命周期协调器。`wifi/` 专注 STA、凭据与重连；`mqtt/` 专注 Broker 客户端；网络管理器根据 Wi-Fi 是否有 IP 决定 MQTT 何时启动和停止。

BLE Bus 与设备协议不直接依赖网络状态：BLE 可继续处理连接/Notify，事件服务按有界队列策略处理数据。

## 关键 API

| API | 用途 |
| --- | --- |
| `network_manager_init()` | 初始化 MQTT 客户端并注册 Wi-Fi 状态回调。 |
| `network_manager_start()` | 启动 Wi-Fi Manager，并按当前连接状态同步 MQTT。 |

声明见 [`network_manager.h`](network_manager.h)。它没有独立网络任务，主要通过 Wi-Fi 状态回调工作。

## 生命周期

```text
main -> network_manager_init -> mqtt_gateway_init
                              -> wifi_manager_init(callback)
     -> network_manager_start -> wifi_manager_start

Wi-Fi 获得 IP -> mqtt_gateway_start
Wi-Fi 失去 IP -> mqtt_gateway_stop
```

Broker 短暂断开后的重试由 ESP-MQTT 处理；STA 断线重连由 `wifi_manager` 处理。网络管理器不缓存业务消息，也不控制 BLE 扫描/连接。

## 排查边界

- Wi-Fi 已连接但 MQTT 不在线：检查 Broker URI、路由可达性、端口、防火墙及 MQTT 日志。
- Wi-Fi 断连时 MQTT stop/start 是生命周期联动，不等价于 BLE 设备断连。
- 修改恢复策略时先区分 STA 问题与 Broker 会话问题，不要把两种重试逻辑混入本模块。
