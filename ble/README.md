# BLE 总线模块

`ble_bus.c/.h` 实现网关作为 BLE Central 的总线层，向上提供设备 Profile 注册、扫描、连接、GATT 发现、Notify 订阅、读写和断线重扫能力。

## 职责

- 根据广播包中的 128 位 Service UUID 匹配 JSON Profile；广播名称只作日志展示。
- 为每个 Profile 维护运行时设备对象和连接句柄，并将 GATT 操作路由到匹配的驱动。
- 校验目标服务/特征 UUID 与属性，订阅通知并分发回调。
- 连接断开后释放运行时资源并继续扫描。
- 通过 `ble_bus_suspend/resume` 支持远程 Profile 事务切换。

## 容量与边界

- Profile 固定槽位上限为 8 个。
- 当前并发 BLE 连接上限为 2 个。
- BLE Bus 不解释厂商帧、不发布 MQTT，也不保存配置。
- `ble_bus_ota_*` 是向 BLE 外设传输升级数据的可选驱动接口；它与 `ota/` 中网关自身 OTA 不同。

## 核心接口

查看 `ble_bus.h`。典型注册顺序是先加载并校验 Profile、注册对应 Driver，再注册 Profile，最后初始化和启动总线。运行时连接信息保存在 bus slot，不写回声明式 JSON。
