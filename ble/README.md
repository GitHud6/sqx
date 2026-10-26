# BLE 总线模块

## 模块定位

`ble/` 是网关 BLE Central 侧的总线抽象。它把 NimBLE 的扫描、连接和 GATT 生命周期封装成设备运行时对象，使配置管理和上层服务面向 Profile 与设备操作，而不直接持有 NimBLE 细节。

该层只处理 BLE 传输生命周期：不解析厂商帧、不决定 MQTT Topic、不持久化设备配置。多个不同外设可共用扫描和连接逻辑，各自的 GATT 行为由 `driver/` 处理，协议帧由 `protocol/` 处理。

## 在项目中的位置

```text
config/ble_config_manager
        | 注册已验证的 Profile + Driver
        v
ble_bus -> NimBLE scan/connect/GATT discovery/notify
   |                    |
   |                    +--> driver/ 回调 -> protocol/
   +--> device state                       |
                                           v
                                 gateway_event -> service/ -> MQTT
```

总线依赖 NimBLE host、`ble_profile_t`/`ble_device_t` 数据结构和 Driver 接口。配置管理负责装载 Profile，再将 Profile 与已注册 Driver 交给总线。

## 关键 API

| API | 用途 |
| --- | --- |
| `ble_bus_register_profile(profile, driver)` | 注册 Profile 及其匹配的 Driver，应在启动扫描前调用。 |
| `ble_bus_init()` | 初始化 NimBLE host/controller 运行时。 |
| `ble_bus_start()` | 启动 Central 扫描与设备发现流程。 |
| `ble_bus_suspend(timeout_ms)` | 暂停扫描并等待连接收敛，用于配置热切换。 |
| `ble_bus_clear_profiles()` | 清除当前运行时 Profile/设备槽位。 |
| `ble_bus_resume()` | Profile 更新后恢复扫描。 |
| `ble_bus_read()` / `ble_bus_write()` | 对已发现设备的特征执行 GATT 读写。 |
| `ble_bus_ota_begin/write/finish()` | 调用 Driver 提供的可选外设 OTA vtable。 |

完整参数、返回值和结构定义以 [`ble_bus.h`](ble_bus.h) 与 [`ble_profile.h`](ble_profile.h) 为准。`ble_bus_ota_*` 是向 BLE 外设传输升级数据的接口，不是网关自身 HTTP OTA；后者见 [`../ota/README.md`](../ota/README.md)。

## 连接与数据流程

1. 配置管理把有效 Profile 和相应 Driver 注册到总线。
2. 总线扫描广播包，以 `advertised_service_uuid` 匹配 Profile；广播名主要用于日志，不作为唯一识别条件。
3. 命中后创建或更新运行时设备对象并发起连接。
4. 连接成功后发现服务和特征，验证 UUID/属性，并订阅配置要求的 Notify 特征。
5. GATT 事件按设备绑定的 Driver 分发；Driver 再把数据交给 Protocol。
6. 断连时释放连接句柄和发现状态，继续扫描以重连或填补连接槽位。

当前 Profile 槽位上限为 8，并发连接上限由 NimBLE 配置控制；项目默认设为 3。Radio、controller 资源和 Wi-Fi coexist 仍会影响实际并发与吞吐。

## 配置热切换与扩展

远程 JSON 更新由 `ble_config_manager` 协调：暂停总线、验证候选 Profile、清理旧槽位、注册新 Profile；失败时恢复旧配置并重启发现。不要由 MQTT 回调直接修改总线内部数组。

新增设备类型应扩展 Profile/Driver/Protocol，不要在总线扫描代码中增加厂商分支。调整并发数时同时检查 `sdkconfig.defaults` 中 NimBLE 最大连接数和 Profile 槽位约束。排查发现失败时核对广播 Service UUID、广告数据、GATT UUID 和属性。

底层状态机见 [`ble_bus.c`](ble_bus.c)，公开接口和回调边界见 [`ble_bus.h`](ble_bus.h)。
