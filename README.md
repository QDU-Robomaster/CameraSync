# CameraSync

MCU 侧相机同步模块。

## 作用

`CameraSync` 订阅一个带 Topic envelope timestamp 的 IMU topic，按 IMU 样本分频触发相机 GPIO，并发布同步结果 topic。

- IMU 驱动只负责正常发布传感器数据
- 相机触发频率只由 `CameraSync.trigger_div` 控制
- 上位机命令只创建一个同步标记：MCU 在下一个反向电平周期开始执行，把这段周期拉长 `div` 倍，然后立刻恢复正常周期
- 完成分频后的第一个有效电平边沿是同步点
- 同步结果 topic 的 envelope timestamp 就是该同步点实际对齐的 IMU 传感器时间

## Topic

输入：

- `imu_topic_name`，默认 `bmi088_gyro`
- `camera_sync_command_topic_name`，默认 `camera_sync_command`

输出：

- `camera_sync_topic_name`，默认 `camera_sync_result`

命令 payload：

```cpp
struct SyncCommand {
  uint32_t div;
  uint32_t active_level;
  uint32_t seq;
};
```

- `div`：单次反向电平周期拉长倍率。`0` 会按 `1` 处理。
- `active_level`：有效触发电平。`0` 表示低有效，非 `0` 表示高有效。
- `seq`：上位机每次同步自增的序号，MCU 在对应同步点原样回传。

输出 payload：

```cpp
struct SyncEvent {
  uint32_t seq;
};
```

实际对齐的 IMU 时间不放在 payload 里，而是使用 topic 消息自带的 timestamp。
普通 GPIO 翻转不发布回执；只有命令对应的同步点发布 `SyncEvent`。

## 时序语义

命令回调不会直接翻 GPIO，也不会直接发布同步结果。它只登记一个 pending 命令。

如果 `active_level=1`，反向电平就是低电平；如果 `active_level=0`，反向电平
就是高电平。每条命令都会等待下一个反向电平周期开始，然后把这个反向周期拉长到
`trigger_div * div` 个 IMU 样本。长周期结束时 GPIO 翻到有效电平，这个边沿就是
同步点，MCU 在同一个 IMU 回调中发布 `SyncEvent{seq}`。随后模块恢复正常
`trigger_div` 周期。

上位机应等待当前 `seq` 回执后再发送下一条同步命令；模块只保留一个 pending 命令，
用于保持 IMU 回调热路径简单且固定。

## 配置示例

```yaml
module: CameraSync
entry_header: Modules/CameraSync/CameraSync.hpp
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_result"
  - imu_topic_name: "bmi088_gyro"
  - trigger_div: 10
  - camera_sync_command_topic_name: "camera_sync_command"
template_args: []
```

## 接入要求

- IMU topic 必须由上游传感器模块用真实采样时间戳发布
- AHRS topic 应继续使用对应 IMU 样本的 timestamp 发布，host 侧按 timestamp 关联
- 本模块不依赖 `BMI088` 内部 callback，也不控制 IMU 发布频率
