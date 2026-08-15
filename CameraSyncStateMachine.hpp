#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace CameraSyncDetail
{

/** @brief CameraSync wire protocol operation. */
enum class Operation : uint8_t
{
  STOP_TRIGGER = 0,   ///< Stop camera trigger output at an IMU sample.
  START_TRIGGER = 1,  ///< Start camera trigger output with a new period.
  FRAME_TRIGGER = 2,  ///< A real camera trigger edge was emitted.
};

/** @brief Host-to-MCU camera trigger command with a fixed 8-byte layout. */
struct SyncCommand
{
  Operation operation = Operation::STOP_TRIGGER;
  uint8_t active_level = 1;
  uint8_t seq = 0;
  uint8_t reserved = 0;
  uint32_t trigger_period_us = 0;
};

/** @brief MCU-to-host command acknowledgement or trigger event. */
struct SyncEvent
{
  uint8_t seq = 0;
  Operation operation = Operation::STOP_TRIGGER;
  uint8_t active_level = 1;
  uint8_t reserved = 0;
  uint32_t effective_period_us = 0;
  uint32_t trigger_sequence = 0;
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

enum class SyncState : uint8_t
{
  RUNNING = 0,
  STOPPED = 1,
};

struct StampedSyncEvent
{
  SyncEvent event{};
  uint64_t timestamp_us = 0;
};

struct SyncActions
{
  static constexpr size_t max_gpio_writes = 2;
  static constexpr size_t max_events = 1;

  std::array<uint8_t, max_gpio_writes> gpio_levels{};
  size_t gpio_write_count = 0;
  std::array<StampedSyncEvent, max_events> events{};
  size_t event_count = 0;

  void WriteGpio(uint8_t level)
  {
    if (gpio_write_count < gpio_levels.size())
    {
      gpio_levels[gpio_write_count++] = level;
    }
  }

  void Publish(const SyncEvent& event, uint64_t timestamp_us)
  {
    if (event_count < events.size())
    {
      events[event_count++] = {.event = event, .timestamp_us = timestamp_us};
    }
  }
};

class StateMachine
{
 public:
  explicit StateMachine(uint32_t default_trigger_period_us)
      : trigger_period_us_(default_trigger_period_us == 0 ? 1U
                                                          : default_trigger_period_us)
  {
  }

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

    const uint64_t elapsed_us = imu_timestamp_us - last_imu_timestamp_us_;
    last_imu_timestamp_us_ = imu_timestamp_us;

    const uint64_t until_trigger_us =
        static_cast<uint64_t>(trigger_period_us_) - phase_elapsed_us_;
    if (elapsed_us < until_trigger_us)
    {
      phase_elapsed_us_ += elapsed_us;
      return actions;
    }

    phase_elapsed_us_ = (elapsed_us - until_trigger_us) % trigger_period_us_;
    EmitFrameTrigger(imu_timestamp_us, actions);
    return actions;
  }

  [[nodiscard]] SyncState State() const { return state_; }
  [[nodiscard]] uint32_t TriggerPeriodUs() const { return trigger_period_us_; }
  [[nodiscard]] uint64_t PhaseElapsedUs() const { return phase_elapsed_us_; }
  [[nodiscard]] uint8_t ActiveLevel() const { return active_level_; }
  [[nodiscard]] uint32_t TriggerSequence() const { return trigger_sequence_; }
  [[nodiscard]] bool PulseActive() const { return pulse_active_; }
  [[nodiscard]] bool CommandPending() const { return pending_command_ready_; }

 private:
  struct CompletedCommand
  {
    bool valid = false;
    SyncCommand command{};
    StampedSyncEvent result{};
  };

  static bool SameCommandKey(const SyncCommand& lhs, const SyncCommand& rhs)
  {
    return lhs.operation == rhs.operation && lhs.seq == rhs.seq;
  }

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

  bool OperationAllowed(Operation operation) const
  {
    switch (operation)
    {
      case Operation::STOP_TRIGGER:
        return state_ == SyncState::RUNNING;
      case Operation::START_TRIGGER:
        return state_ == SyncState::STOPPED;
      case Operation::FRAME_TRIGGER:
        return false;
    }
    return false;
  }

  void FinishPulse(SyncActions& actions)
  {
    if (!pulse_active_)
    {
      return;
    }
    pulse_active_ = false;
    actions.WriteGpio(static_cast<uint8_t>(!active_level_));
  }

  void RebasePhase(uint64_t imu_timestamp_us)
  {
    phase_initialized_ = true;
    last_imu_timestamp_us_ = imu_timestamp_us;
    phase_elapsed_us_ = 0;
  }

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

  void RememberCompleted(const SyncCommand& command, const SyncEvent& event,
                         uint64_t timestamp_us)
  {
    last_completed_command_ = {
        .valid = true,
        .command = command,
        .result = {.event = event, .timestamp_us = timestamp_us},
    };
  }

  uint32_t trigger_period_us_ = 1;
  uint64_t phase_elapsed_us_ = 0;
  uint64_t last_imu_timestamp_us_ = 0;
  uint8_t active_level_ = 1;
  uint8_t active_start_seq_ = 0;
  uint32_t trigger_sequence_ = 0;
  bool phase_initialized_ = false;
  bool pulse_active_ = false;

  bool pending_command_ready_ = false;
  SyncCommand pending_command_{};
  SyncState state_ = SyncState::RUNNING;
  CompletedCommand last_completed_command_{};
};

}  // namespace CameraSyncDetail
