# CameraSync

由带时间戳 IMU 消息驱动的 MCU 侧相机周期触发模块 / MCU-side camera trigger Module driven by timestamped IMU messages

## 1. 模块作用 / Purpose

CameraSync 以 IMU Topic 消息的时间戳为时钟，按周期在一个 GPIO 上输出相机触发脉冲。每个实际输出的触发边沿发布一个 `FRAME_TRIGGER` 事件，事件时间戳为产生该边沿的 IMU 消息时间戳，上位机据此得到每帧图像的触发时刻。上位机通过命令 Topic 发送 `STOP_TRIGGER` 与 `START_TRIGGER`，每条命令在下一条 IMU 消息处生效并回复一个 ACK。

构造时，GPIO 配置为推挽输出并写低电平（与有效电平无关，第一条命令生效时按命令中的有效电平写无效电平），随后订阅 IMU Topic 与命令 Topic。两个订阅都是同步回调，在发布者的线程中执行，状态机由一个 Mutex 串行访问，所以这两个 Topic 须在线程上下文中发布。GPIO 写入与事件发布在释放锁之后执行，事件通过 `PublishFromCallback` 发布。

CameraSync uses the timestamps of the IMU Topic messages as its clock and outputs a camera trigger pulse on one GPIO every period. Each trigger edge actually emitted publishes a `FRAME_TRIGGER` event stamped with the time of the IMU message that produced it, from which the host obtains the trigger time of every image. The host sends `STOP_TRIGGER` and `START_TRIGGER` on the command Topic; each command takes effect at the next IMU message and is answered with an ACK.

Upon construction, the GPIO is configured as push-pull output and driven low (regardless of the active level; the first command to apply writes the inactive level of the active level it carries), and the IMU Topic and the command Topic are subscribed. Both subscriptions are synchronous callbacks that run on the publisher's thread, and the state machine is accessed under one Mutex, so both Topics are published from thread context. GPIO writes and event publication run after the lock is released, and events are published with `PublishFromCallback`.

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

上电后模块处于 `RUNNING`，有效电平为高，构造参数 `trigger_period_us` 是默认周期。第一条 IMU 消息作为周期起点；自该时间戳起满一个周期后，第一条到达或越过期限的 IMU 消息产生 GPIO 边沿。

每个实际输出的边沿同时发布一个 `FRAME_TRIGGER`：

- 事件的 Topic 时间戳是产生边沿的 IMU 消息时间戳。
- `effective_period_us` 是当前运行周期。
- `seq` 是最近一次生效的 `START_TRIGGER.seq`，上电默认运行阶段为 `0`。
- `trigger_sequence` 从 `1` 开始，每个边沿加一，按 `uint32_t` 回绕。

触发脉冲在下一条 IMU 消息到达时恢复到无效电平，所以脉冲宽度为一个 IMU 间隔。边沿只能落在 IMU 消息上，平均周期等于 `trigger_period_us`，单个间隔以 IMU 间隔为粒度变化。一条 IMU 消息跨过多个周期时，模块只产生一个边沿，`trigger_sequence` 加一，余下的时间计入下一个周期。IMU 时间戳倒退时，模块不产生边沿，并以该消息作为新的周期起点。

`STOP_TRIGGER` 在命令后的下一条 IMU 消息处生效：GPIO 保持无效电平，之后不再触发，ACK 的 `effective_period_us` 为 `0`，`trigger_sequence` 是停止前最后一个边沿的序号。同一条 IMU 消息上恰好有边沿到期时，STOP 优先，该边沿不产生。状态已经是 `STOPPED` 时，带新 `seq` 的 STOP 仍在下一条 IMU 消息处重新写入无效电平并回复新的 ACK，重启后的上位机据此完成 STOP/START 握手。

`START_TRIGGER` 仅在 `STOPPED` 状态有效，同样在下一条 IMU 消息处设置新周期和有效电平并回复 ACK。这条 IMU 消息不产生边沿，ACK 的 `trigger_sequence` 为 `0`；第一个边沿出现在自 ACK 时间戳起满一个 `trigger_period_us` 之后，序号从 `1` 重新开始。

After power-up the Module is in `RUNNING` with the active level high, and the constructor parameter `trigger_period_us` is the default period. The first IMU message starts the period; once one period has elapsed from its timestamp, the first IMU message that reaches or passes the deadline produces the GPIO edge.

Every edge actually emitted also publishes one `FRAME_TRIGGER`:

- The Topic timestamp of the event is the timestamp of the IMU message that produced the edge.
- `effective_period_us` is the current running period.
- `seq` is the `seq` of the most recently applied `START_TRIGGER`; it is `0` in the default running phase after power-up.
- `trigger_sequence` starts at `1`, grows by one with each edge and wraps as a `uint32_t`.

The trigger pulse returns to the inactive level when the next IMU message arrives, so a pulse lasts one IMU interval. Edges fall on IMU messages: the mean period equals `trigger_period_us` while single intervals vary in steps of the IMU interval. When one IMU message spans several periods, the Module produces one edge, `trigger_sequence` grows by one, and the remaining time counts toward the next period. When the IMU timestamp moves backwards, the Module produces no edge and starts a new period at that message.

`STOP_TRIGGER` takes effect at the next IMU message after the command: the GPIO stays at the inactive level, no further edges follow, the ACK carries `effective_period_us` of `0`, and `trigger_sequence` is the number of the last edge before the stop. When an edge is due on the same IMU message, STOP takes precedence and that edge is not produced. When the state is already `STOPPED`, a STOP with a new `seq` still rewrites the inactive level at the next IMU message and is answered with a new ACK, so that a restarted host can complete the STOP/START handshake.

`START_TRIGGER` is valid in the `STOPPED` state only. It also takes effect at the next IMU message, which sets the new period and active level and is answered with an ACK. That IMU message produces no edge and the ACK carries `trigger_sequence` of `0`; the first edge appears once a full `trigger_period_us` has elapsed from the ACK timestamp, and the numbering restarts from `1`.

## 4. 幂等性 / Idempotence

命令的幂等键为 `{operation, seq}`。模块保存一条待生效命令和最近一条已生效的命令：

- 与待生效命令同键的副本不再登记，副本中的其他字段可以不同。
- 已有待生效命令时，不同键的新命令被丢弃，不回复 ACK。
- 与最近一条已生效命令同键的副本使模块重发该命令的原 ACK，时间戳也是原来的。
- 重发 ACK 不重复 GPIO 操作，触发相位和边沿序号保持不变。
- `FRAME_TRIGGER` 事件不改变缓存的 ACK。
- `STOPPED` 状态下带新序号的 STOP 按新命令处理。

模块要求命令通道保持顺序。上位机在收到当前 `seq` 的 ACK 后发送下一条命令；超时时重发字段完全相同的当前命令，重试次数有限。

IMU Topic 以传感器采样时间戳发布。相机丢帧由相机帧自身的时间戳差值判断；`FRAME_TRIGGER` 用于把图像对应到 MCU 实际发出的触发边沿。

The idempotence key of a command is `{operation, seq}`. The Module keeps one pending command and the last applied command:

- A copy of the pending command with the same key is not registered again; the other fields of the copy may differ.
- While a command is pending, a new command with a different key is discarded without an ACK.
- A copy of the last applied command with the same key makes the Module resend that command's original ACK, with the original timestamp.
- A resent ACK repeats no GPIO operation and leaves the trigger phase and the edge numbering unchanged.
- `FRAME_TRIGGER` events do not change the cached ACK.
- In the `STOPPED` state, a STOP with a new sequence number is processed as a new command.

The Module requires the command channel to preserve ordering. The host sends the next command after receiving the ACK of the current `seq`; on timeout it resends the current command with identical fields, with a limited number of retries.

The IMU Topic is published with the sensor sampling timestamp. Camera frame drops are determined from the differences between the timestamps of the camera frames themselves; `FRAME_TRIGGER` matches images to the trigger edges actually emitted by the MCU.

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
- `trigger_period_us`：上电默认触发周期，单位 µs，默认 `50000`（20 Hz），需非零（`REQUIRE`，Release 下同样检查）。
- `camera_sync_command_topic_name`：上位机控制命令的 Topic 名称，默认 `"camera_sync_command"`。

Dependencies:

- `camera_pin`: `LibXR::GPIO`, the pin wired to the hardware trigger input of the camera.

Configuration parameters (`Param`):

- `camera_sync_topic_name`: name of the output Topic for ACKs and edge events, default `"camera_sync_result"`.
- `imu_topic_name`: name of the IMU Topic used as the time base, default `"bmi088_gyro"`.
- `trigger_period_us`: default trigger period after power-up in µs, default `50000` (20 Hz); must be non-zero (`REQUIRE`, checked in Release builds as well).
- `camera_sync_command_topic_name`: name of the host command Topic, default `"camera_sync_command"`.

## 6. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.imu_topic_name`（默认 `bmi088_gyro`） | 订阅 | `Eigen::Matrix<float, 3, 1>` | 时间基准 IMU 消息，只使用时间戳，数据内容不参与计算 |
| `param.camera_sync_command_topic_name`（默认 `camera_sync_command`） | 订阅 | `CameraSync::SyncCommand` | 上位机的 `STOP_TRIGGER` / `START_TRIGGER` 命令 |
| `param.camera_sync_topic_name`（默认 `camera_sync_result`） | 发布 | `CameraSync::SyncEvent` | 命令 ACK 与每个触发边沿的 `FRAME_TRIGGER` |

三个 Topic 都通过 `CreateTopic` 按名称查找或创建，已存在时类型保持一致。

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.imu_topic_name` (default `bmi088_gyro`) | Subscribe | `Eigen::Matrix<float, 3, 1>` | IMU message used as the time base; only its timestamp is used |
| `param.camera_sync_command_topic_name` (default `camera_sync_command`) | Subscribe | `CameraSync::SyncCommand` | `STOP_TRIGGER` / `START_TRIGGER` commands from the host |
| `param.camera_sync_topic_name` (default `camera_sync_result`) | Publish | `CameraSync::SyncEvent` | Command ACKs and the `FRAME_TRIGGER` of every trigger edge |

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

`tests/` 测试状态机 `CameraSyncStateMachine.hpp`（仅使用标准库）。主机构建打开 `BUILD_TESTING` 时模块会加入 `camera_sync_state_machine_test`，模块 CI 运行它；MCU 交叉编译时不加入。也可以作为独立的 CMake 工程运行：

```sh
cmake -S tests -B build/tests
cmake --build build/tests
ctest --test-dir build/tests
```

`tests/` tests the state machine `CameraSyncStateMachine.hpp` (standard library only). A host build with `BUILD_TESTING` adds `camera_sync_state_machine_test`, which Module CI runs; cross builds for the MCU leave it out. It also runs as a standalone CMake project with the commands in the code block above.
