# SD 卡驱动

## 目标

本模块只负责 RayCube 物理 SDMMC 接口的初始化、FAT 文件系统挂载和卸载。
它不读取 G-code，不解析 `.gc` 文件，也不直接调用 `execute_line()` 或 Planner。

## RayCube 硬件配置

当前采用 ESP32-S3 SDMMC 1-bit 模式：

| 信号 | GPIO |
| --- | ---: |
| SD 检测 | 14 |
| CMD | 11 |
| CLK | 12 |
| D0 | 13 |

挂载点固定为 `/sd`，挂载失败时不格式化 SD 卡。

## 接口

- `sdcard_start()`：挂载 SD 卡；已挂载时返回 `ESP_ERR_INVALID_STATE`。
- `sdcard_stop()`：卸载 SD 卡；未挂载时返回 `ESP_ERR_INVALID_STATE`。
- `sdcard_is_mounted()`：查询当前挂载状态。

## 当前边界

`app_main()` 暂不自动调用 `sdcard_start()`，避免没有插卡时影响现有电机和串口启动流程。
后续 SD 作业模块应在确认挂载成功后，通过文件读取接口向唯一的 G-code 执行路径提交完整命令行。

## 验收标准

1. 插入 RayCube SD 卡后，`sdcard_start()` 能挂载到 `/sd`。
2. 未插卡或引脚配置错误时，返回明确的 ESP-IDF 错误码，不格式化介质。
3. `sdcard_stop()` 能释放挂载资源，重复启动和重复停止不会静默改变状态。
4. 本驱动不改变 UART、Planner、Stepper 和激光现有调用链。
