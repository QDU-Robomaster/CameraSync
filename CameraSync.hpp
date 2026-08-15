#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 由带时间戳 IMU 消息驱动的 MCU 侧相机周期触发模块
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_result"
  - imu_topic_name: "bmi088_gyro"
  - trigger_period_us: 50000
  - camera_sync_command_topic_name: "camera_sync_command"
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstddef>
#include <cstdint>

#include "CameraSyncStateMachine.hpp"
#include "app_framework.hpp"
#include "gpio.hpp"
#include "libxr.hpp"
#include "transform.hpp"

/**
 * @brief MCU 侧相机周期触发模块。
 * @details 模块以 IMU Topic envelope timestamp 推进触发周期。STOP_TRIGGER 和
 *          START_TRIGGER 在下一条 IMU 消息处生效并回执；每个真实 GPIO 触发边沿
 *          都发布 FRAME_TRIGGER，事件 timestamp 即产生边沿的 IMU 时间戳。
 */
class CameraSync : public LibXR::Application
{
 public:
  using ImuSample = Eigen::Matrix<float, 3, 1>;
  using Operation = CameraSyncDetail::Operation;
  using SyncCommand = CameraSyncDetail::SyncCommand;
  using SyncEvent = CameraSyncDetail::SyncEvent;

  /**
   * @brief 构造 CameraSync 模块。
   * @param hw 硬件容器。
   * @param app 应用管理器。
   * @param camera_pin_name 相机触发 GPIO 名称。
   * @param camera_sync_topic_name 同步事件 Topic 名称。
   * @param imu_topic_name 作为时间基准的 IMU Topic 名称。
   * @param trigger_period_us 上电默认触发周期，单位微秒，必须非零。
   * @param camera_sync_command_topic_name 上位机控制命令 Topic 名称。
   */
  CameraSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* camera_pin_name, const char* camera_sync_topic_name,
             const char* imu_topic_name, uint32_t trigger_period_us,
             const char* camera_sync_command_topic_name)
      : camera_sync_pin_(*hw.template FindOrExit<LibXR::GPIO>({camera_pin_name})),
        imu_topic_(LibXR::Topic::CreateTopic<ImuSample>(imu_topic_name)),
        command_topic_(
            LibXR::Topic::CreateTopic<SyncCommand>(camera_sync_command_topic_name)),
        camera_sync_topic_(LibXR::Topic::CreateTopic<SyncEvent>(camera_sync_topic_name)),
        state_machine_(trigger_period_us)
  {
    ASSERT(trigger_period_us != 0);

    camera_sync_pin_.SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                                .pull = LibXR::GPIO::Pull::NONE});
    camera_sync_pin_.Write(false);

    imu_callback_ = LibXR::Topic::Callback::Create(
        [](bool in_isr, CameraSync* self, LibXR::MicrosecondTimestamp timestamp,
           const ImuSample&) { self->OnImuMessage(in_isr, timestamp); },
        this);
    imu_topic_.RegisterCallback(imu_callback_);

    command_callback_ = LibXR::Topic::Callback::Create(
        [](bool in_isr, CameraSync* self, LibXR::MicrosecondTimestamp,
           const SyncCommand& command) { self->OnCommand(in_isr, command); },
        this);
    command_topic_.RegisterCallback(command_callback_);

    app.Register(*this);
  }

  /** @brief CameraSync 当前不输出周期监控。 */
  void OnMonitor() override {}

 private:
  void OnCommand(bool in_isr, const SyncCommand& command)
  {
    CameraSyncDetail::SyncActions actions;
    {
      LibXR::Mutex::LockGuard lock(state_machine_mutex_);
      actions = state_machine_.OnCommand(command);
    }
    ApplyActions(actions, in_isr);
  }

  void OnImuMessage(bool in_isr, LibXR::MicrosecondTimestamp imu_timestamp)
  {
    CameraSyncDetail::SyncActions actions;
    {
      LibXR::Mutex::LockGuard lock(state_machine_mutex_);
      actions = state_machine_.OnImu(static_cast<uint64_t>(imu_timestamp));
    }
    ApplyActions(actions, in_isr);
  }

  void ApplyActions(const CameraSyncDetail::SyncActions& actions, bool in_isr)
  {
    for (size_t i = 0; i < actions.gpio_write_count; ++i)
    {
      camera_sync_pin_.Write(actions.gpio_levels[i] != 0);
    }
    for (size_t i = 0; i < actions.event_count; ++i)
    {
      SyncEvent event = actions.events[i].event;
      camera_sync_topic_.PublishFromCallback(
          event, LibXR::MicrosecondTimestamp(actions.events[i].timestamp_us), in_isr);
    }
  }

  LibXR::GPIO& camera_sync_pin_;

  LibXR::Topic imu_topic_;
  LibXR::Topic command_topic_;
  LibXR::Topic camera_sync_topic_;
  LibXR::Topic::Callback imu_callback_;
  LibXR::Topic::Callback command_callback_;

  LibXR::Mutex state_machine_mutex_{};
  CameraSyncDetail::StateMachine state_machine_;
};
