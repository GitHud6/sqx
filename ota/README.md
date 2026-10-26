# 网关 OTA 模块

`ota_manager.c/.h` 实现网关自身固件 OTA。升级命令通过 MQTT 收到，实际固件通过 HTTP 从文件服务器下载；这与 BLE Driver 的外设 OTA vtable 不同。

## 命令与状态

向默认命令 Topic `netgate/ota` 发布：

```json
{"url":"http://192.168.2.10:8000/netgate_hello_world.bin"}
```

状态发布到 `netgate/ota/status`，含 `state`、`error`、`received_bytes` 和 `total_bytes`。OTA 任务使用单项队列串行执行，不在 MQTT 事件回调中下载。

## 更新流程

1. 校验命令 JSON 和 HTTP URL。
2. 获取非活动 OTA app 分区，读取 HTTP 200 响应及确定的 Content-Length，并检查大小不超过分区容量。
3. 流式下载并写入 OTA 分区。
4. `esp_ota_end()` 校验 ESP 镜像，再设置新的启动分区并重启。
5. 新应用完成核心服务初始化后调用 `esp_ota_mark_app_valid_cancel_rollback()`；未确认镜像在下次启动时由 bootloader 回滚。

## 安全与限制

- 只支持明文 `http://`，不支持 TLS、重定向、未知长度/分块传输、签名或来源认证。
- ESP 镜像格式校验不是数字签名校验。任何能向 OTA Topic 发布命令并控制 HTTP 文件内容的一方，都可能替换固件。
- 仅在可信隔离的学习网络使用。生产部署应自行集成 TLS、签名验证、身份授权和防回滚策略。
- 此实现刻意不内置 TLS、CA 或设备凭据；使用者可按自己的服务端环境扩展传输保护。
- 代码尚未按项目规则执行编译或硬件 OTA 验证；首次使用前由开发者在硬件上验证成功升级和失败回滚。
