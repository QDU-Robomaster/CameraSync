#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

/**
 * @brief CameraSync 的通信协议与触发状态机。只用标准库，可在主机上测试；上位机的
 *        CameraFrameSync 也引用这里的协议类型。
 *        Wire protocol and trigger state machine of CameraSync. Standard library only,
 *        so it is testable on the host; the host-side CameraFrameSync uses the same
 *        protocol types.
 */
namespace CameraSyncDetail
{

/**
 * @brief CameraSync 通信协议的操作码。
 *        CameraSync wire protocol operation.
 */
enum class Operation : uint8_t
{
  STOP_TRIGGER = 0,   ///< 在下一条 IMU 样本处停止触发输出
                      ///< Stop trigger output at the next IMU sample
  START_TRIGGER = 1,  ///< 以新周期启动触发输出
                      ///< Start trigger output with a new period
  FRAME_TRIGGER = 2,  ///< 已发出一个真实触发边沿
                      ///< A real trigger edge was emitted
};

/**
 * @brief 上位机到 MCU 的触发命令，固定 8 字节布局。
 *        Host-to-MCU trigger command with a fixed 8-byte layout.
 *
 * @details `seq` 非零，与 `operation` 组成幂等键；`reserved` 须为 0；
 *          `trigger_period_us` 对 START_TRIGGER 非零，对 STOP_TRIGGER 为 0。
 *          `seq` is non-zero and forms the idempotence key together with `operation`;
 *          `reserved` must be 0; `trigger_period_us` is non-zero for START_TRIGGER and
 *          0 for STOP_TRIGGER.
 */
struct SyncCommand
{
  Operation operation = Operation::STOP_TRIGGER;  ///< 操作码 Operation
  uint8_t active_level = 1;                       ///< 有效电平 Active level (0 or 1)
  uint8_t seq = 0;                                ///< 命令序号 Command sequence number
  uint8_t reserved = 0;                           ///< 保留 Reserved
  uint32_t trigger_period_us = 0;                 ///< 触发周期 Trigger period (us)
};

/**
 * @brief MCU 到上位机的命令 ACK 或触发边沿事件，固定 12 字节布局。
 *        MCU-to-host command acknowledgement or trigger event with a fixed 12-byte
 *        layout.
 *
 * @details `seq` 在 ACK 中是所答复命令的序号，在 FRAME_TRIGGER 中是最近生效的
 *          START_TRIGGER 序号；`effective_period_us` 停止时为 0；`trigger_sequence`
 *          计数真实触发边沿，START_TRIGGER 的 ACK 中为 0。
 *          In an ACK, `seq` is the sequence number of the answered command; in a
 *          FRAME_TRIGGER it is the sequence number of the most recently applied
 *          START_TRIGGER. `effective_period_us` is 0 when stopped. `trigger_sequence`
 *          counts real trigger edges and is 0 in the ACK of START_TRIGGER.
 */
struct SyncEvent
{
  uint8_t seq = 0;                                ///< 命令序号 Command sequence number
  Operation operation = Operation::STOP_TRIGGER;  ///< 操作码 Operation
  uint8_t active_level = 1;                       ///< 有效电平 Active level
  uint8_t reserved = 0;                           ///< 保留 Reserved
  uint32_t effective_period_us = 0;               ///< 生效周期 Effective period (us)
  uint32_t trigger_sequence = 0;                  ///< 边沿计数 Edge count
};

static_assert(sizeof(SyncCommand) == 8);
static_assert(sizeof(SyncEvent) == 12);
static_assert(std::is_standard_layout_v<SyncCommand>);
static_assert(std::is_standard_layout_v<SyncEvent>);
static_assert(std::is_trivially_copyable_v<SyncCommand>);
static_assert(std::is_trivially_copyable_v<SyncEvent>);
static_assert(offsetof(SyncCommand, operation) == 0);
static_assert(offsetof(SyncCommand, active_level) == 1);
static_assert(offsetof(SyncCommand, seq) == 2);
static_assert(offsetof(SyncCommand, reserved) == 3);
static_assert(offsetof(SyncCommand, trigger_period_us) == 4);
static_assert(offsetof(SyncEvent, seq) == 0);
static_assert(offsetof(SyncEvent, operation) == 1);
static_assert(offsetof(SyncEvent, active_level) == 2);
static_assert(offsetof(SyncEvent, reserved) == 3);
static_assert(offsetof(SyncEvent, effective_period_us) == 4);
static_assert(offsetof(SyncEvent, trigger_sequence) == 8);

/**
 * @brief 触发输出的运行状态。
 *        Run state of the trigger output.
 */
enum class SyncState : uint8_t
{
  RUNNING = 0,  ///< 按周期输出触发边沿 / Trigger edges are emitted every period
  STOPPED = 1,  ///< 不输出边沿，引脚保持无效电平 / No edges; the pin stays inactive
};

/**
 * @brief 带时间戳的事件，时间戳为产生该事件的 IMU 样本时间。
 *        Event with the timestamp of the IMU sample that produced it.
 */
struct StampedSyncEvent
{
  SyncEvent event{};          ///< ACK 或边沿事件 / ACK or edge event
  uint64_t timestamp_us = 0;  ///< IMU 样本时间 (µs) / IMU sample time (µs)
};

/**
 * @brief 状态机一次调用要求调用方执行的动作，按记录顺序执行：先写 GPIO，再发布事件。
 *        Actions one state-machine call asks the caller to perform, in recorded order:
 *        GPIO writes first, then events.
 *
 * @details 容量按最坏情况取定：一次 `OnImu` 最多先结束上一个脉冲、再写一次新电平（两次
 *          GPIO），最多一个事件。超出容量的记录被丢弃，状态机不会产生这种情况。
 *          Capacity covers the worst case: one `OnImu` ends the previous pulse and then
 *          writes one new level (two GPIO writes), with at most one event. Records
 *          beyond capacity are dropped; the state machine never produces them.
 */
struct SyncActions
{
  static constexpr size_t max_gpio_writes = 2;  ///< GPIO 写入容量 / GPIO write capacity
  static constexpr size_t max_events = 1;       ///< 事件容量 / Event capacity

  std::array<uint8_t, max_gpio_writes> gpio_levels{};  ///< 依次写入的电平 / Levels
  size_t gpio_write_count = 0;                         ///< 有效写入数 / Valid writes
  std::array<StampedSyncEvent, max_events> events{};   ///< 依次发布的事件 / Events
  size_t event_count = 0;                              ///< 有效事件数 / Valid events

  /// 记录一次 GPIO 写入 / Record one GPIO write.
  void WriteGpio(uint8_t level)
  {
    if (gpio_write_count < gpio_levels.size())
    {
      gpio_levels[gpio_write_count++] = level;
    }
  }

  /// 记录一个要发布的事件 / Record one event to publish.
  void Publish(const SyncEvent& event, uint64_t timestamp_us)
  {
    if (event_count < events.size())
    {
      events[event_count++] = {.event = event, .timestamp_us = timestamp_us};
    }
  }
};

/**
 * @brief 相机触发状态机：以 IMU 样本时间为时钟，不直接访问硬件。
 *        Camera trigger state machine clocked by IMU sample times; it touches no
 *        hardware.
 *
 * @details 时间只在 `OnImu` 中推进。触发边沿只能落在 IMU 样本上：累计经过的时间达到一个
 *          周期就在该样本输出边沿，超出的部分（最多半个 IMU 间隔）结转到下一周期，所以
 *          平均周期接近设定值，单个间隔以 IMU 间隔为粒度变化，且不短于周期减半个 IMU 间隔。边沿保持到下一个样本，脉冲宽度为一个 IMU 间隔。命令先
 *          登记，在下一个 IMU 样本生效并回 ACK。上电即以默认周期运行，边沿事件的 `seq`
 *          为 0，直到第一条 START_TRIGGER 生效。
 *          Time advances only in `OnImu`. Edges fall on IMU samples: once the elapsed
 *          time reaches a period an edge is emitted on that sample and the excess, at most
 *          half an IMU interval, carries into the next period, so the mean period stays
 *          close to the setting while single intervals vary in steps of the IMU interval
 *          and are never shorter than the period minus half an IMU interval. An edge lasts until the next sample, so the
 *          pulse is one IMU interval wide. A command is held and applied, with its ACK,
 *          at the next IMU sample. After power-up the machine runs at the default
 *          period, and edge events carry `seq` 0 until the first START_TRIGGER applies.
 *
 *          不是线程安全的；调用方串行化 `OnCommand` 与 `OnImu`（CameraSync 用互斥锁）。
 *          Not thread-safe; the caller serialises `OnCommand` and `OnImu` (CameraSync
 *          holds a mutex).
 */
class StateMachine
{
 public:
  /**
   * @brief 以默认周期构造，处于 RUNNING。
   *        Construct running at the default period.
   * @param default_trigger_period_us 默认周期 (µs)；0 按 1 µs 处理，CameraSync 在构造时
   *                                  拒绝 0。
   *                                  Default period (µs); 0 is treated as 1 µs, and
   *                                  CameraSync rejects 0 at construction.
   */
  explicit StateMachine(uint32_t default_trigger_period_us)
      : trigger_period_us_(default_trigger_period_us == 0 ? 1U
                                                          : default_trigger_period_us)
  {
  }

  /**
   * @brief 登记一条上位机命令，本次调用不改变触发输出。
   *        Register one host command; this call does not change the trigger output.
   *
   * @details 以 (`operation`, `seq`) 为幂等键：
   *          - 与待生效命令相同：忽略（等待它生效）；
   *          - 与最近一条已生效命令相同：立即重发该命令的原 ACK（含原时间戳），状态不变；
   *          - 字段无效、已有待生效命令，或 RUNNING 时收到 START_TRIGGER：
   *            丢弃且不回 ACK，由上位机超时重发；
   *          - 否则登记，在下一个 `OnImu` 生效。
   *          The idempotence key is (`operation`, `seq`):
   *          - same as the pending command: ignored (it is about to apply);
   *          - same as the last applied command: its original ACK, with its original
   *            timestamp, is published again and the state does not change;
   *          - invalid fields, a command already pending, or START_TRIGGER while
   *            RUNNING: dropped without an ACK; the host resends on timeout;
   *          - otherwise registered, to apply at the next `OnImu`.
   * @return 要执行的动作（只可能是重发的 ACK）。
   *         Actions to perform (only a replayed ACK is possible).
   */
  SyncActions OnCommand(const SyncCommand& command)
  {
    SyncActions actions;
    if (pending_command_ready_ && SameCommandKey(pending_command_, command))
    {
      return actions;
    }

    if (last_completed_command_.valid &&
        SameCommandKey(last_completed_command_.command, command))
    {
      actions.Publish(last_completed_command_.result.event,
                      last_completed_command_.result.timestamp_us);
      return actions;
    }

    if (!ValidCommand(command) || pending_command_ready_ ||
        !OperationAllowed(command.operation))
    {
      return actions;
    }

    pending_command_ = command;
    pending_command_ready_ = true;
    return actions;
  }

  /**
   * @brief 以一个 IMU 样本推进状态机。
   *        Advance the state machine by one IMU sample.
   *
   * @details 依次：结束上一个脉冲；有待生效命令则执行它并发 ACK，本样本不再输出边沿；
   *          否则在 RUNNING 下累计时间，到周期时输出边沿并发布 FRAME_TRIGGER。样本时间
   *          倒退或刚开始运行时只重定相位，不输出边沿。两个样本之间跨过多个周期时只输出
   *          一个边沿，`trigger_sequence` 只加一，没有发出的边沿不补发。
   *          In order: end the previous pulse; if a command is pending, apply it and
   *          publish its ACK, with no edge on this sample; otherwise, while RUNNING,
   *          accumulate time and emit an edge with a FRAME_TRIGGER when a period is
   *          reached. A sample time that goes backwards, or the first sample of a run,
   *          only re-bases the phase without an edge. When several periods pass between
   *          two samples only one edge is emitted and `trigger_sequence` grows by one;
   *          missed edges are not made up.
   * @param imu_timestamp_us IMU 样本时间 (µs)。IMU sample time (µs).
   * @return 要执行的动作。Actions to perform.
   */
  SyncActions OnImu(uint64_t imu_timestamp_us)
  {
    SyncActions actions;
    FinishPulse(actions);

    if (pending_command_ready_)
    {
      const SyncCommand command = pending_command_;
      pending_command_ready_ = false;

      if (command.operation == Operation::STOP_TRIGGER)
      {
        ApplyStop(command, imu_timestamp_us, actions);
      }
      else if (command.operation == Operation::START_TRIGGER)
      {
        ApplyStart(command, imu_timestamp_us, actions);
      }
      return actions;
    }

    if (state_ == SyncState::STOPPED)
    {
      return actions;
    }

    if (!phase_initialized_ || imu_timestamp_us < last_imu_timestamp_us_)
    {
      RebasePhase(imu_timestamp_us);
      return actions;
    }

    // 到周期就出边沿，超出的部分结转到下一周期，但最多结转半个 IMU 间隔：时间戳抖动可能
    // 让边沿晚一个样本，全额结转会让下一个边沿只隔一个样本，相机来不及响应。跨过多个周期
    // 时也不补发缺失的边沿。
    // Emit an edge when a period is reached and carry the excess into the next period,
    // but at most half an IMU interval: timestamp jitter can delay an edge by one
    // sample, and carrying all of it would put the next edge one sample later, too soon
    // for the camera. Missed edges are not made up when several periods passed.
    const uint64_t elapsed_us = imu_timestamp_us - last_imu_timestamp_us_;
    last_imu_timestamp_us_ = imu_timestamp_us;

    const uint64_t until_trigger_us =
        static_cast<uint64_t>(trigger_period_us_) - phase_elapsed_us_;
    if (elapsed_us < until_trigger_us)
    {
      phase_elapsed_us_ += elapsed_us;
      return actions;
    }

    const uint64_t excess_us = (elapsed_us - until_trigger_us) % trigger_period_us_;
    phase_elapsed_us_ = excess_us < elapsed_us / 2 ? excess_us : elapsed_us / 2;
    EmitFrameTrigger(imu_timestamp_us, actions);
    return actions;
  }

  /// 运行状态 / Run state.
  [[nodiscard]] SyncState State() const { return state_; }
  /// 当前周期 (µs) / Current period (µs).
  [[nodiscard]] uint32_t TriggerPeriodUs() const { return trigger_period_us_; }
  /// 本周期已经过的时间 (µs) / Time elapsed in the current period (µs).
  [[nodiscard]] uint64_t PhaseElapsedUs() const { return phase_elapsed_us_; }
  /// 有效电平 / Active level.
  [[nodiscard]] uint8_t ActiveLevel() const { return active_level_; }
  /// 本次运行已输出的边沿数 / Edges emitted in the current run.
  [[nodiscard]] uint32_t TriggerSequence() const { return trigger_sequence_; }
  /// 引脚是否处于有效电平 / Whether the pin is at the active level.
  [[nodiscard]] bool PulseActive() const { return pulse_active_; }
  /// 是否有待生效的命令 / Whether a command is waiting to apply.
  [[nodiscard]] bool CommandPending() const { return pending_command_ready_; }

 private:
  /// 最近一条已生效命令及其 ACK，用于重发 / Last applied command and its ACK, for
  /// replay.
  struct CompletedCommand
  {
    bool valid = false;
    SyncCommand command{};
    StampedSyncEvent result{};
  };

  // 幂等键只比较操作码与序号；其余字段不同的重发视为同一条命令。
  // The idempotence key compares only operation and seq; a resend with other fields
  // changed counts as the same command.
  static bool SameCommandKey(const SyncCommand& lhs, const SyncCommand& rhs)
  {
    return lhs.operation == rhs.operation && lhs.seq == rhs.seq;
  }

  // 字段约束见 README 的命令表；FRAME_TRIGGER 只作为事件，不能作为命令。
  // Field constraints as in the README command table; FRAME_TRIGGER is an event only.
  static bool ValidCommand(const SyncCommand& command)
  {
    if (command.active_level > 1 || command.seq == 0 || command.reserved != 0)
    {
      return false;
    }

    switch (command.operation)
    {
      case Operation::STOP_TRIGGER:
        return command.trigger_period_us == 0;
      case Operation::START_TRIGGER:
        return command.trigger_period_us != 0;
      case Operation::FRAME_TRIGGER:
        return false;
    }
    return false;
  }

  // STOP 任何时候都可以；START 只能在停止后（重启必须先 STOP）。
  // STOP is always allowed; START only when stopped (a restart is STOP then START).
  bool OperationAllowed(Operation operation) const
  {
    switch (operation)
    {
      case Operation::STOP_TRIGGER:
        return true;
      case Operation::START_TRIGGER:
        return state_ == SyncState::STOPPED;
      case Operation::FRAME_TRIGGER:
        return false;
    }
    return false;
  }

  // 每个样本先结束上一个脉冲，所以脉冲宽度为一个 IMU 间隔。
  // Every sample first ends the previous pulse, so a pulse lasts one IMU interval.
  void FinishPulse(SyncActions& actions)
  {
    if (!pulse_active_)
    {
      return;
    }
    pulse_active_ = false;
    actions.WriteGpio(static_cast<uint8_t>(!active_level_));
  }

  // 以该样本为周期起点：下一个边沿在一个完整周期之后。
  // Make this sample the start of a period: the next edge comes one full period later.
  void RebasePhase(uint64_t imu_timestamp_us)
  {
    phase_initialized_ = true;
    last_imu_timestamp_us_ = imu_timestamp_us;
    phase_elapsed_us_ = 0;
  }

  // 停止并写无效电平；ACK 带停止前最后一个边沿的序号。已停止时收到新序号的 STOP 也走
  // 这里，重写无效电平并回新 ACK，重启后的上位机据此重新握手。
  // Stop and write the inactive level; the ACK carries the number of the last edge
  // before the stop. A STOP with a new seq while already stopped also comes here and
  // rewrites the inactive level with a new ACK, so a restarted host can handshake again.
  void ApplyStop(const SyncCommand& command, uint64_t imu_timestamp_us,
                 SyncActions& actions)
  {
    active_level_ = command.active_level;
    state_ = SyncState::STOPPED;
    phase_initialized_ = false;
    phase_elapsed_us_ = 0;
    pulse_active_ = false;
    actions.WriteGpio(static_cast<uint8_t>(!active_level_));

    const SyncEvent event{.seq = command.seq,
                          .operation = Operation::STOP_TRIGGER,
                          .active_level = active_level_,
                          .reserved = 0,
                          .effective_period_us = 0,
                          .trigger_sequence = trigger_sequence_};
    actions.Publish(event, imu_timestamp_us);
    RememberCompleted(command, event, imu_timestamp_us);
  }

  // 安装新周期与有效电平，边沿序号清零，以 ACK 样本为相位起点；ACK 样本本身不出边沿。
  // Install the new period and active level, reset the edge count and start the phase at
  // the ACK sample; the ACK sample itself emits no edge.
  void ApplyStart(const SyncCommand& command, uint64_t imu_timestamp_us,
                  SyncActions& actions)
  {
    active_level_ = command.active_level;
    trigger_period_us_ = command.trigger_period_us;
    active_start_seq_ = command.seq;
    trigger_sequence_ = 0;
    state_ = SyncState::RUNNING;
    pulse_active_ = false;
    RebasePhase(imu_timestamp_us);
    actions.WriteGpio(static_cast<uint8_t>(!active_level_));

    const SyncEvent event{.seq = command.seq,
                          .operation = Operation::START_TRIGGER,
                          .active_level = active_level_,
                          .reserved = 0,
                          .effective_period_us = trigger_period_us_,
                          .trigger_sequence = 0};
    actions.Publish(event, imu_timestamp_us);
    RememberCompleted(command, event, imu_timestamp_us);
  }

  // 边沿事件带最近一条生效 START 的序号，上位机据此丢弃别的运行（如重启前）的边沿。
  // Edge events carry the seq of the START in effect, so the host can drop edges of
  // another run (for example from before a restart).
  void EmitFrameTrigger(uint64_t imu_timestamp_us, SyncActions& actions)
  {
    actions.WriteGpio(active_level_);
    pulse_active_ = true;
    ++trigger_sequence_;

    const SyncEvent event{.seq = active_start_seq_,
                          .operation = Operation::FRAME_TRIGGER,
                          .active_level = active_level_,
                          .reserved = 0,
                          .effective_period_us = trigger_period_us_,
                          .trigger_sequence = trigger_sequence_};
    actions.Publish(event, imu_timestamp_us);
  }

  // 只保留最近一条：上位机收到当前命令的 ACK 后才发下一条，重发只针对最近一条。
  // Only the latest is kept: the host sends the next command after the ACK of the
  // current one, so resends only concern the latest.
  void RememberCompleted(const SyncCommand& command, const SyncEvent& event,
                         uint64_t timestamp_us)
  {
    last_completed_command_ = {
        .valid = true,
        .command = command,
        .result = {.event = event, .timestamp_us = timestamp_us},
    };
  }

  uint32_t trigger_period_us_ = 1;      ///< 当前周期 (µs) / Current period (µs)
  uint64_t phase_elapsed_us_ = 0;       ///< 本周期已过时间 / Elapsed in period
  uint64_t last_imu_timestamp_us_ = 0;  ///< 上一个样本时间 / Previous sample time
  uint8_t active_level_ = 1;            ///< 有效电平 / Active level
  uint8_t active_start_seq_ = 0;        ///< 生效 START 的序号 / Applied START seq
  uint32_t trigger_sequence_ = 0;       ///< 本次运行边沿数 / Edges in this run
  bool phase_initialized_ = false;      ///< 相位已对齐 / Phase based on a sample
  bool pulse_active_ = false;           ///< 引脚为有效电平 / Pin at active level

  bool pending_command_ready_ = false;         ///< 有待生效命令 / Command pending
  SyncCommand pending_command_{};              ///< 待生效命令 / Pending command
  SyncState state_ = SyncState::RUNNING;       ///< 运行状态 / Run state
  CompletedCommand last_completed_command_{};  ///< 用于重发 ACK / For ACK replay
};

}  // namespace CameraSyncDetail
