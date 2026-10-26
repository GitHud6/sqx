# 配置模块

配置层负责解析设备 JSON、校验 Profile、绑定 Driver 与 callback_set，并维护配置更新事务。

## JSON 与持久化

- `devices.json` 是内置启动配置，并由 ESP-IDF 构建嵌入固件。
- schema_version 当前为 3；最多允许 8 个设备 Profile。
- `ble_config_manager.c` 在独立任务内处理 MQTT 配置候选，执行校验、运行时切换、持久化和回滚。
- `ble_config_store.c` 将 active、backup、candidate、revision、CRC32 和更新时间保存在 NVS。
- `ble_profile.c` 维护活动与备用 Profile Bank。

配置 MQTT Topic 默认是 `netgate/config/ble`，结果为 `netgate/config/ble/result`。特征 `topic` 是 MQTT Topic 的单级片段。只有注册表中存在相应 Driver 和 callback_set 时配置才会激活。

CRC 用于检测意外损坏，不是密码学签名，不能证明配置来自可信用户。
