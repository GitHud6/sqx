# BLE 配置模块

## 模块定位

`config/` 把声明式 JSON 转换为可运行的 BLE Profile，并负责配置更新事务和 NVS 持久化。它连接“远程可配置设备定义”和“固定编译进固件的 Driver/Protocol 实现”：UUID、特征属性、解码参数和 Topic 可以配置；任意厂商私有算法仍需预先编译进固件。

## 文件职责

- `devices.json`：内置启动配置样例，构建时嵌入 S3 固件。
- `ble_profile.c/.h`：解析 JSON、校验结构并维护活动/候选 Profile Bank。
- `ble_config_manager.c/.h`：协调 MQTT 候选配置、运行时切换、事务结果和回滚。
- `ble_config_store.c/.h`：以 NVS blob 保存活动配置、备份、候选版本和校验元数据。

## 关键 API

### Profile 层

| API | 用途 |
| --- | --- |
| `ble_profile_load()` | 从已持久化配置或嵌入 JSON 装载活动 Profile。 |
| `ble_profile_get_embedded_json()` | 获取固件内置基准 JSON。 |
| `ble_profile_stage_json(json, length)` | 解析并暂存候选配置，不立即替换活动配置。 |
| `ble_profile_staged_count()` / `ble_profile_count()` | 查询候选/活动 Profile 数量。 |
| `ble_profile_commit_staged()` | 将候选 Bank 提升为活动 Bank。 |
| `ble_profile_rollback()` / `ble_profile_discard_staged()` | 回滚或丢弃候选配置。 |

### 管理与存储层

| API | 用途 |
| --- | --- |
| `ble_config_manager_load_initial()` | 启动时读取、验证并将活动 Profile 注册到 Driver Registry 和 BLE Bus。 |
| `ble_config_manager_init()` | 注册 MQTT 配置处理器并启动候选配置处理任务。 |
| `ble_config_store_load_active()` / `load_backup()` | 读取 NVS 活动配置或备份。 |
| `ble_config_store_stage()` / `commit()` | 写入候选配置并提交为活动版本。 |
| `ble_config_store_restore_backup()` / `discard_candidate()` | 恢复备份或丢弃候选配置。 |

完整声明见 [`ble_profile.h`](../ble/ble_profile.h)、[`ble_config_manager.h`](ble_config_manager.h) 和 [`ble_config_store.h`](ble_config_store.h)。

## 与其他模块的联动

```text
MQTT config topic -> mqtt_gateway callback -> ble_config_manager task
 -> ble_profile parse/validate -> driver_registry lookup
 -> ble_bus suspend/clear/register/resume -> config_store NVS
 -> mqtt_gateway publish transaction result
```

BLE Bus 不读 NVS，MQTT 不解析 Profile。配置管理是两者之间的事务协调者；候选配置在独立任务中处理，避免在 ESP-MQTT 事件回调中执行 BLE 重配和 NVS 写入。

## JSON 与校验要点

- 当前 `schema_version` 为 3，最多 8 个设备 Profile；字段定义和完整实例以 [`devices.json`](devices.json) 为准。
- `advertised_service_uuid` 用于匹配广播；`services[].uuid`、characteristic UUID/属性用于连接后 GATT 发现校验。
- `driver` 和 `callback_set` 必须能在固件注册表中找到；JSON 不能动态创建任意 C 函数指针。
- `topic` 是 MQTT 主题单级片段；通用 decoder 元数据支持 raw/整数、字节序、偏移、比例和单位等简单格式。
- 复杂私有帧仍需在 `protocol/` 增加 C 实现，并由 Driver callback set 调用。
- NVS CRC32 用于意外损坏检测，不是来源认证或数字签名。

增加 schema 字段时应同步更新解析、范围/组合校验、JSON 示例和文档，并考虑旧配置兼容及运行时回滚；不要只改 JSON 示例而不扩展解析器。

实现分别见 [`ble_profile.c`](ble_profile.c)、[`ble_config_manager.c`](ble_config_manager.c) 和 [`ble_config_store.c`](ble_config_store.c)。
