# CameraSync

由带时间戳 IMU 消息驱动的 MCU 侧相机周期触发模块 / MCU-side camera trigger Module driven by timestamped IMU messages

## 1. 模块作用 / Purpose

CameraSync 以 IMU Topic 消息的 envelope timestamp 推进相机触发周期，通过一个 GPIO 输出触发脉冲。上位机通过命令 Topic 发送 `STOP_TRIGGER` 与 `START_TRIGGER` 两种命令，每条命令在下一条 IMU 消息处生效并回复一个 ACK；每产生一个真实的 GPIO 触发边沿，发布一个 `FRAME_TRIGGER` 事件。

构造时，GPIO 配置为推挽输出并写低电平，随后订阅 IMU Topic 与命令 Topic。两个订阅都使用同步回调，状态机由一个 Mutex 串行访问；GPIO 写入与事件发布在释放状态锁之后执行，事件通过 `PublishFromCallback` 发布。

CameraSync measures the camera trigger period with the envelope timestamp of the IMU Topic messages and outputs the trigger pulse on one GPIO. The host sends `STOP_TRIGGER` and `START_TRIGGER` commands on the command Topic; each command takes effect at the next IMU message and is answered with an ACK. For every real GPIO trigger edge a `FRAME_TRIGGER` event is published.

Upon construction, the GPIO is configured as push-pull output and driven low, and the IMU Topic and the command Topic are subscribed. Both subscriptions use synchronous callbacks and the state machine is accessed under one Mutex; GPIO writes and event publishing run after the state lock is released, and events are published with `PublishFromCallback`.

## 2. 通信协议 / Wire Protocol

上位机到 MCU 的命令固定为 8 字节：

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

MCU 到上位机的 ACK 或边沿事件固定为 12 字节：

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

命令的字段约束如下，不满足约束的命令被丢弃：

| operation | 有效状态 | seq | reserved | trigger_period_us |
| --- | --- | ---: | ---: | ---: |
| `STOP_TRIGGER` | `RUNNING` 或 `STOPPED` | 非零 | `0` | `0` |
| `START_TRIGGER` | `STOPPED` | 非零 | `0` | 非零 |
| `FRAME_TRIGGER` | 仅作为事件 | - | - | - |

`active_level` 取 `0` 或 `1`。多字节字段使用运行平台的原生字节序（小端平台）。

The host-to-MCU command has a fixed size of 8 bytes, and the MCU-to-host ACK or edge event has a fixed size of 12 bytes. The layouts are the `SyncCommand` and `SyncEvent` structures listed above.

The command fields are constrained as follows; a command that violates the constraints is discarded:

| operation | Valid state | seq | reserved | trigger_period_us |
| --- | --- | ---: | ---: | ---: |
| `STOP_TRIGGER` | `RUNNING` or `STOPPED` | non-zero | `0` | `0` |
| `START_TRIGGER` | `STOPPED` | non-zero | `0` | non-zero |
| `FRAME_TRIGGER` | event only | - | - | - |

`active_level` is `0` or `1`. Multi-byte fields use the native byte order of the platform (little-endian platforms).

## 3. 时序 / Timing

上电后模块处于 `RUNNING`，有效电平为高，构造参数 `trigger_period_us` 是默认周期。第一条 IMU 消息建立时间相位；自该 timestamp 起满一个周期后，第一条达到或越过期限的 IMU 消息产生真实 GPIO 边沿。

每个真实边沿同时发布一个 `FRAME_TRIGGER`：

- Topic envelope timestamp 是产生边沿的 IMU timestamp。
- `effective_period_us` 是当前运行周期。
- `seq` 是最近一次生效的 `START_TRIGGER.seq`，上电默认运行阶段为 `0`。
- `trigger_sequence` 从 `1` 开始，按真实边沿递增，按 `uint32_t` 回绕。

触发脉冲在下一条 IMU 消息到达时恢复到无效电平。一条 IMU 消息跨过多个理想周期时，模块产生一个真实边沿，`trigger_sequence` 加 `1`，内部相位推进到该 timestamp 之后的下一个周期。IMU timestamp 回退时，模块不产生边沿，并以该消息重新建立相位。

`STOP_TRIGGER` 在命令后的下一条 IMU 消息处生效：GPIO 保持无效电平，后续不再触发，ACK 的 `effective_period_us` 为 `0`，`trigger_sequence` 是停止前最后一个真实边沿的序号。同一条 IMU 消息恰好到期时，STOP 优先，该边沿不产生。状态已经是 `STOPPED` 时，带新 `seq` 的 STOP 仍在下一条 IMU 消息处重新写入无效电平并返回新的 ACK，重启后的上位机据此完成 STOP/START 握手。

`START_TRIGGER` 仅在 `STOPPED` 状态有效，同样在下一条 IMU 消息处安装新周期和有效电平并返回 ACK。ACK 样本不产生边沿，`trigger_sequence` 为 `0`；第一个边沿出现在自 ACK timestamp 起满一个 `trigger_period_us` 之后，序号从 `1` 重新开始。

After power-up the Module is in `RUNNING` with the active level high, and the constructor parameter `trigger_period_us` is the default period. The first IMU message establishes the time phase; once one period has elapsed from that timestamp, the first IMU message that reaches or passes the deadline produces the real GPIO edge.

Every real edge also publishes one `FRAME_TRIGGER`:

- The Topic envelope timestamp is the IMU timestamp that produced the edge.
- `effective_period_us` is the current running period.
- `seq` is the `seq` of the most recently applied `START_TRIGGER`; it is `0` in the default running phase after power-up.
- `trigger_sequence` starts at `1`, increments with each real edge and wraps as a `uint32_t`.

The trigger pulse returns to the inactive level when the next IMU message arrives. When one IMU message spans several ideal periods, the Module produces one real edge, `trigger_sequence` increases by `1`, and the internal phase advances to the next period after that timestamp. When the IMU timestamp moves backwards, the Module produces no edge and re-establishes the phase from that message.

`STOP_TRIGGER` takes effect at the next IMU message after the command: the GPIO stays at the inactive level, no further edges are produced, the ACK carries `effective_period_us` of `0`, and `trigger_sequence` is the number of the last real edge before the stop. When the same IMU message is exactly due, STOP takes precedence and that edge is not produced. When the state is already `STOPPED`, a STOP with a new `seq` still rewrites the inactive level at the next IMU message and returns a new ACK, so that a restarted host can complete the STOP/START handshake.

`START_TRIGGER` is valid in the `STOPPED` state only. It also takes effect at the next IMU message, where it installs the new period and active level and returns an ACK. The ACK sample produces no edge and carries `trigger_sequence` of `0`; the first edge appears once a full `trigger_period_us` has elapsed from the ACK timestamp, and the sequence restarts from `1`.

## 4. 幂等性 / Idempotence

命令的幂等键为 `{operation, seq}`。模块保存一个 pending 命令和最后一个已完成的命令：

- pending 命令的相同键副本不重新排队，副本中的其他字段可以不同。
- 已有 pending 命令时，不同键的新命令被丢弃。
- 最后一个已完成命令的相同键副本触发 ACK 重放，重放原 ACK 和原 envelope timestamp。
- ACK 重放不重复 GPIO 操作，触发相位和边沿序号保持不变。
- `FRAME_TRIGGER` 是遥测事件，最后一个已完成命令的 ACK 保持不变。
- `STOPPED` 状态下带新序号的 STOP 是新的 desired-state 命令，按新命令处理。

模块要求命令通道保持顺序。上位机在收到当前 `seq` 的 ACK 后发送下一条操作；超时时重发字段完全相同的当前命令，重试次数有界。

IMU Topic 以传感器采样 timestamp 发布。相机丢帧通过相机帧自身的 timestamp 差值判断；`FRAME_TRIGGER` 用于关联 MCU 实际发出的触发边沿。

The idempotence key of a command is `{operation, seq}`. The Module keeps one pending command and the last completed command:

- A copy of the pending command with the same key is not queued again; the other fields of the copy may differ.
- While a command is pending, a new command with a different key is discarded.
- A copy of the last completed command with the same key triggers an ACK replay of the original ACK and the original envelope timestamp.
- An ACK replay repeats no GPIO operation and leaves the trigger phase and the edge sequence unchanged.
- `FRAME_TRIGGER` is a telemetry event; the ACK of the last completed command stays unchanged.
- In the `STOPPED` state, a STOP with a new sequence number is a new desired-state command and is processed as a new command.

The Module requires the command channel to preserve ordering. The host sends the next operation after receiving the ACK of the current `seq`; on timeout it resends the current command with identical fields, with a bounded number of retries.

The IMU Topic is published with the sensor sampling timestamp. Camera frame drops are determined from the differences between the timestamps of the camera frames themselves; `FRAME_TRIGGER` associates the trigger edges actually emitted by the MCU.

## 5. 构造接口 / Constructor

```cpp
CameraSync(LibXR::GPIO& camera_pin,
           const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `camera_pin`：`LibXR::GPIO`，连接相机硬件触发输入的引脚。

配置参数（`Param`）：

- `camera_sync_topic_name`：ACK 与边沿事件的输出 Topic 名称，默认 `"camera_sync_result"`。
- `imu_topic_name`：作为时间基准的 IMU Topic 名称，默认 `"bmi088_gyro"`。
- `trigger_period_us`：上电默认触发周期，单位 µs，默认 `50000`（20 Hz），需非零（`ASSERT`）。
- `camera_sync_command_topic_name`：上位机控制命令的 Topic 名称，默认 `"camera_sync_command"`。

Dependencies:

- `camera_pin`: `LibXR::GPIO`, the pin wired to the hardware trigger input of the camera.

Configuration parameters (`Param`):

- `camera_sync_topic_name`: name of the output Topic for ACKs and edge events, default `"camera_sync_result"`.
- `imu_topic_name`: name of the IMU Topic used as the time base, default `"bmi088_gyro"`.
- `trigger_period_us`: default trigger period after power-up in µs, default `50000` (20 Hz); must be non-zero (`ASSERT`).
- `camera_sync_command_topic_name`: name of the host command Topic, default `"camera_sync_command"`.

## 6. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.imu_topic_name`（默认 `bmi088_gyro`） | 订阅 | `Eigen::Matrix<float, 3, 1>` | 时间基准 IMU 消息，使用 timestamp，数据内容不参与计算 |
| `param.camera_sync_command_topic_name`（默认 `camera_sync_command`） | 订阅 | `CameraSync::SyncCommand` | 上位机的 `STOP_TRIGGER` / `START_TRIGGER` 命令 |
| `param.camera_sync_topic_name`（默认 `camera_sync_result`） | 发布 | `CameraSync::SyncEvent` | 命令 ACK 与每个真实触发边沿的 `FRAME_TRIGGER` |

三个 Topic 都通过 `CreateTopic` 按名称查找或创建，已存在时类型保持一致。

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.imu_topic_name` (default `bmi088_gyro`) | Subscribe | `Eigen::Matrix<float, 3, 1>` | IMU message used as the time base; its timestamp is used and the data content takes no part in the computation |
| `param.camera_sync_command_topic_name` (default `camera_sync_command`) | Subscribe | `CameraSync::SyncCommand` | `STOP_TRIGGER` / `START_TRIGGER` commands from the host |
| `param.camera_sync_topic_name` (default `camera_sync_result`) | Publish | `CameraSync::SyncEvent` | Command ACKs and the `FRAME_TRIGGER` of every real trigger edge |

All three Topics are looked up by name or created with `CreateTopic`; an existing Topic has the same type.

## 7. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/CameraSync` 写入的实例，`camera_pin` 填写为 BSP 中通过 `XR_REGISTER`（硬件注册）注册的 GPIO 名称，Topic 名称与 IMU 实例发布的名称相同：

An instance written by `xrobot instance add QDU-Robomaster/CameraSync`, with `camera_pin` set to the GPIO name registered in the BSP with `XR_REGISTER` (Registration), and the IMU Topic name matching the name published by the IMU instance:

```yaml
modules:
  - module: QDU-Robomaster/CameraSync
    id: camera_sync
    args:
      - camera_pin: CAMERA
      - param:
          camera_sync_topic_name: "camera_sync_result"
          imu_topic_name: "gimbal_gyro"
          trigger_period_us: 20000
          camera_sync_command_topic_name: "camera_sync_command"
```

## 8. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR（IMU 样本类型 `Eigen::Matrix` 来自 LibXR 自带的 Eigen）。

硬件：一个 GPIO 输出，连接相机的硬件触发输入；一个以传感器采样时间戳发布 `Eigen::Matrix<float, 3, 1>` 的 IMU Topic。

Dependencies: LibXR (the `Eigen::Matrix` IMU sample type comes from the Eigen bundled with LibXR).

Hardware: one GPIO output wired to the hardware trigger input of the camera, and an IMU Topic that publishes `Eigen::Matrix<float, 3, 1>` with the sensor sampling timestamp.

## 9. 测试 / Tests

`tests/` 是独立的 CMake 工程，测试状态机 `CameraSyncStateMachine.hpp`（仅使用标准库），命令如下：

```sh
cmake -S tests -B build/tests
cmake --build build/tests
ctest --test-dir build/tests
```

`tests/` is a standalone CMake project that tests the state machine `CameraSyncStateMachine.hpp` (standard library only), with the commands in the code block above.
