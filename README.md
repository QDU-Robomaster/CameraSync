# CameraSync

`CameraSync` 在 MCU 侧使用 IMU Topic 的 envelope timestamp 推进相机触发周期。模块只包含
`STOP_TRIGGER`、`START_TRIGGER` 两条控制命令，并为每个真实 GPIO 触发边沿发布
`FRAME_TRIGGER` 事件。

模块没有工作线程、Semaphore、探针流程或重置协议。IMU 和命令 Topic 都使用同步回调；
状态机访问由一个 Mutex 串行化，GPIO 写入和事件发布在状态锁释放后执行。

## Topic

输入：

- `imu_topic_name`，默认 `bmi088_gyro`
- `camera_sync_command_topic_name`，默认 `camera_sync_command`

输出：

- `camera_sync_topic_name`，默认 `camera_sync_result`

## Wire protocol

Host 到 MCU 的命令固定为 8 字节：

```cpp
enum class Operation : uint8_t {
  STOP_TRIGGER = 0,
  START_TRIGGER = 1,
  FRAME_TRIGGER = 2,
};

struct SyncCommand {
  Operation operation;
  uint8_t active_level;
  uint8_t seq;
  uint8_t reserved;
  uint32_t trigger_period_us;
};
```

MCU 到 Host 的 ACK 或边沿事件固定为 12 字节：

```cpp
struct SyncEvent {
  uint8_t seq;
  Operation operation;
  uint8_t active_level;
  uint8_t reserved;
  uint32_t effective_period_us;
  uint32_t trigger_sequence;
};
```

字段约束：

| operation | 有效状态 | seq | reserved | trigger_period_us |
| --- | --- | ---: | ---: | ---: |
| `STOP_TRIGGER` | `RUNNING` 或 `STOPPED` | 非零 | `0` | `0` |
| `START_TRIGGER` | `STOPPED` | 非零 | `0` | 非零 |
| `FRAME_TRIGGER` | 不能作为命令 | - | - | - |

`active_level` 只接受 `0` 或 `1`。所有多字节字段使用运行平台的原生小端布局。

## Timing

上电后模块处于 `RUNNING`，构造参数 `trigger_period_us` 是默认周期。第一条 IMU 消息只建立
时间相位；从该 timestamp 起满一个周期后，第一条达到或越过期限的 IMU 消息产生真实
GPIO 边沿。

每个真实边沿同时发布一个 `FRAME_TRIGGER`：

- Topic envelope timestamp 是产生边沿的 IMU timestamp
- `effective_period_us` 是当前运行周期
- `seq` 是最近一次生效的 `START_TRIGGER.seq`；上电默认运行阶段为 `0`
- `trigger_sequence` 从 `1` 开始，只按真实边沿递增，并按 `uint32_t` 自然回绕

触发脉冲在下一条 IMU 消息到达时恢复到无效电平。若一条 IMU 消息跨过多个理想周期，
模块只产生一个真实边沿，不补发历史脉冲，也不为漏过的理想周期增加
`trigger_sequence`；内部相位会推进到当前 timestamp 之后的下一个周期。若 IMU timestamp
回退，模块不产生边沿，并以该消息重新建立相位。

`STOP_TRIGGER` 在命令后的下一条 IMU 消息处生效：GPIO 保持无效，后续不再触发，ACK 的
`effective_period_us` 为 `0`，`trigger_sequence` 是停止前最后一个真实边沿序号。若同一条
IMU 消息本来恰好到期，STOP 优先，不产生该边沿。即使状态已经是 `STOPPED`，带新 `seq`
的 STOP 仍会在下一条 IMU 消息重新确认无效电平并返回新 ACK，使重启后的 Host 能从
STOP/START 握手恢复。

`START_TRIGGER` 仅在 `STOPPED` 有效，也在下一条 IMU 消息处安装新周期并 ACK。ACK 样本
不会产生边沿，`trigger_sequence` 为 `0`；第一条边沿必须等待从 ACK timestamp 起满一个
完整的 `trigger_period_us`，随后序号从 `1` 重新开始。

## Idempotence

命令幂等键为 `{operation, seq}`。模块只保留一个 pending 命令和最后一个已完成命令：

- pending 命令的相同键副本不重新排队，即使副本中的其他字段不同
- 最后一个已完成命令的相同键副本只重放原 ACK 和原 envelope timestamp
- ACK 重放不重复 GPIO 操作，也不重置触发相位或边沿序号
- `FRAME_TRIGGER` 是遥测事件，不会覆盖最后一个已完成命令的 ACK
- `STOPPED` 状态下的新 STOP 序号是新的 desired-state 命令，不属于 ACK 重放

命令通道必须保持顺序。Host 应等待当前 `seq` 的 ACK 后再发送下一条操作；超时只重发
字段完全相同的当前命令，且重试次数应有界。

## Configuration

```yaml
module: CameraSync
entry_header: Modules/CameraSync/CameraSync.hpp
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_result"
  - imu_topic_name: "bmi088_gyro"
  - trigger_period_us: 50000
  - camera_sync_command_topic_name: "camera_sync_command"
template_args: []
```

IMU Topic 必须使用传感器采样 timestamp 发布。相机丢帧判断应比较相机帧自身的 timestamp
差值；`FRAME_TRIGGER` 用于关联 MCU 实际发出的触发边沿，不应代替相机时间线。
