# CameraSync

MCU 侧相机同步模块。

## 作用

`CameraSync` 订阅一个带 Topic envelope timestamp 的 IMU topic，按 IMU 样本分频触发相机 GPIO，并发布同步结果 topic。

- IMU 驱动只负责正常发布传感器数据
- 相机触发频率只由 `CameraSync.trigger_div` 控制
- 同步结果 topic 的 envelope timestamp 就是本次触发对齐的 IMU 传感器时间

## Topic

输入：

- `imu_topic_name`，默认 `bmi088_gyro`

输出：

- `camera_sync_topic_name`，默认 `camera_sync_result`

输出 payload：

```cpp
struct SyncEvent {
  uint32_t trigger_sequence;
  uint32_t imu_sequence;
  uint8_t pin_level;
  uint8_t status;
  uint16_t reserved;
};
```

其中实际对齐的 IMU 时间不放在 payload 里，而是使用 topic 消息自带的 timestamp。

## 配置示例

```yaml
module: CameraSync
entry_header: Modules/CameraSync/CameraSync.hpp
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_result"
  - imu_topic_name: "bmi088_gyro"
  - trigger_div: 10
template_args: []
```

## 接入要求

- IMU topic 必须由上游传感器模块用真实采样时间戳发布
- AHRS topic 应继续使用对应 IMU 样本的 timestamp 发布，host 侧按 timestamp 关联
- 本模块不依赖 `BMI088` 内部 callback，也不控制 IMU 发布频率
