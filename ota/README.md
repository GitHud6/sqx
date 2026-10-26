# 网关 OTA 模块

## 模块定位

`ota/ota_manager.c/.h` 负责网关自身固件升级：接收 MQTT OTA 命令、通过 HTTP 流式下载到非活动 app 分区、校验 ESP 镜像格式并切换启动分区。它不实现 BLE 外设升级；后者属于 Driver vtable 的另一条数据路径。

## 关键 API

| API | 用途 |
| --- | --- |
| `ota_manager_init()` | 创建单项 OTA 请求队列/工作任务，并把命令处理器注册给 MQTT。 |
| `ota_manager_confirm_running_image()` | 核心服务初始化成功后确认新镜像，取消 bootloader 回滚状态。 |
| `ota_manager_reject_running_image()` | 启动检查失败时拒绝 pending 镜像并请求回滚。 |
| `mqtt_gateway_publish_ota_status()` | OTA 工作流使用的状态发布接口，定义于 MQTT 模块。 |

接口声明见 [`ota_manager.h`](ota_manager.h)，Topic 配置见 [`../main/Kconfig.projbuild`](../main/Kconfig.projbuild)。

## 升级时序

```text
Broker -> mqtt_gateway -> OTA callback -> OTA queue/task
       -> HTTP client -> inactive OTA partition -> esp_ota_end
       -> set boot partition -> reboot
       -> main initializes core services -> confirm image
                                      failure -> reject/bootloader rollback
```

命令提供固件 URL。下载在独立任务执行，避免阻塞 MQTT 事件回调。状态覆盖排队、下载、失败和准备重启等阶段，并报告接收字节数。

## 分区依赖

OTA 要求分区表提供 `otadata` 和至少两个可切换的 app 槽位；本项目定义于 [`../partitions.csv`](../partitions.csv)，Flash 默认容量见 [`../sdkconfig.defaults`](../sdkconfig.defaults)。镜像不能超过目标非活动分区容量。

## 安全边界和验证

- 当前仅支持明文 HTTP，要求 HTTP 200 和可确定的 Content-Length；不支持 TLS、重定向、未知长度传输、签名或来源认证。
- `esp_ota_end()` 是镜像格式校验，不是发布者真实性校验。能发布 OTA 命令并控制 URL 内容的一方可能替换固件。
- 本项目有意不内置使用者自己的 TLS CA、密钥或 Broker 凭据。
- 手工验证至少覆盖成功升级、下载中断、HTTP 错误、镜像损坏和新固件初始化失败回滚；本次文档工作不执行编译或烧录。

MQTT 命令和状态示例见根目录 [`../README.md`](../README.md)。
