# 主程序与构建集成模块

## 模块定位

`main/` 是应用组合根：按依赖顺序初始化各模块、连接回调并处理启动失败，不承载 BLE、Wi-Fi、MQTT 或 OTA 的业务实现。阅读 `app_main()` 可以了解系统如何由独立模块组装。

## 启动编排

当前主要顺序如下：

1. 初始化 NVS 并准备 OTA 启动确认流程。
2. 创建 `gateway_service` 事件队列和发布任务。
3. 注册内置通用/设备 Driver。
4. 初始化 BLE Bus，读取并注册内置或 NVS 中的 Profile。
5. 初始化并启动 BLE 配网 Peripheral，再启动 Central 总线。
6. 初始化远程 BLE 配置管理，使 MQTT 配置进入配置事务。
7. 初始化网络管理并启动 Wi-Fi；取得 IP 后才连接 MQTT。
8. 核心服务启动成功后确认 pending OTA 镜像。

该顺序表达模块依赖。新镜像需等核心初始化成功后确认，避免尚不能工作的固件被永久标记有效。

## 入口与关键配置

| 文件/入口 | 职责 |
| --- | --- |
| `main.c:app_main()` | 初始化模块、注入回调、处理启动依赖。 |
| `Kconfig.projbuild` | Wi-Fi 初始参数、MQTT Broker/Topic、OTA 命令大小等。 |
| `CMakeLists.txt` | 注册 C 源文件、include 路径、IDF 组件依赖并嵌入 `config/devices.json`。 |
| `../sdkconfig.defaults` | S3、NimBLE、Flash 与分区表默认配置。 |
| `../partitions.csv` | NVS、OTA data 和 app 槽位布局。 |

`app_main()` 由 ESP-IDF 启动框架调用，不是外部普通 API。各模块公开接口见对应目录 README。

## 联动关系

```text
main/app_main
 +-- config + driver registry + ble_bus
 +-- provisioning + wifi_manager + network_manager
 +-- mqtt_gateway + gateway_service
 +-- ota_manager
```

主程序负责连接模块边界：BLE 事件通过 service 到 MQTT；MQTT 配置/OTA 命令分别进入 Config Manager/OTA Manager。不要把遥测解析或 JSON schema 代码堆到 `main.c`。

## 修改指南

- 新增模块时添加自己的 `.c/.h` 和 README，在 `CMakeLists.txt` 注册，由 `main.c` 调公开初始化函数。
- 新增启动依赖时调整顺序，并说明失败后是终止启动还是降级运行。
- 新增 Kconfig 时标注开发默认值，不能把个人凭据写入公开配置。
- 修改 Flash 布局时同时检查 Flash size、CSV、OTA slot 容量和 sdkconfig 默认值。

项目规则禁止自动构建和烧录；人工验证流程见根 README。
