#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 由带时间戳 IMU 消息驱动的 MCU 侧相机周期触发模块 / MCU-side camera trigger Module driven by timestamped IMU messages
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstddef>
#include <cstdint>

#include "CameraSyncStateMachine.hpp"
#include "gpio.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "transform.hpp"

/**
 * @brief MCU 侧相机周期触发模块。
 *        MCU-side periodic camera trigger Module.
 *
 * @details 模块以 IMU Topic envelope timestamp 推进触发周期。STOP_TRIGGER 和
 *          START_TRIGGER 在下一条 IMU 消息处生效并回执；每个真实 GPIO 触发边沿
 *          都发布 FRAME_TRIGGER，事件 timestamp 即产生边沿的 IMU 时间戳。
 *          The Module advances the trigger period with the envelope timestamp of the
 *          IMU Topic. STOP_TRIGGER and START_TRIGGER take effect at the next IMU
 *          message and are acknowledged; every real GPIO trigger edge publishes a
 *          FRAME_TRIGGER whose timestamp is the IMU timestamp that produced the edge.
 */
class CameraSync
{
 public:
  using ImuSample = Eigen::Matrix<float, 3, 1>;   ///< IMU Topic 消息类型 IMU message type
  using Operation = CameraSyncDetail::Operation;  ///< 协议操作码 Protocol operation
  using SyncCommand = CameraSyncDetail::SyncCommand;  ///< 上位机命令 Host command
  using SyncEvent = CameraSyncDetail::SyncEvent;  ///< ACK 或边沿事件 ACK or edge event

  /**
   * @brief CameraSync 配置参数。
   *        CameraSync configuration parameters.
   */
  struct Param
  {
    const char* camera_sync_topic_name;  ///< ACK 与边沿事件的输出 Topic 名称
    ///< Name of the output Topic for ACKs and edge events
    const char* imu_topic_name;  ///< 作为时间基准的 IMU Topic 名称
    ///< Name of the IMU Topic used as the time base
    uint32_t trigger_period_us;  ///< 上电默认触发周期 (µs)，必须非零
    ///< Default trigger period after power-up (µs), must be non-zero
    const char* camera_sync_command_topic_name;  ///< 上位机控制命令的 Topic 名称
    ///< Name of the host command Topic
  };

  /**
   * @brief 构造 CameraSync，把 GPIO 配置为推挽输出并写低电平，订阅 IMU 与命令 Topic。
   *        Construct CameraSync: configure the GPIO as push-pull output driven low and
   *        subscribe to the IMU and command Topics.
   *
   * @param camera_pin 连接相机硬件触发输入的 GPIO。
   *                   GPIO wired to the hardware trigger input of the camera.
   * @param param 配置参数；trigger_period_us 为 0 时触发 ASSERT。
   *              Configuration parameters; ASSERT fails when trigger_period_us is 0.
   */
  CameraSync(
      LibXR::GPIO& camera_pin,
      const Param& param = {.camera_sync_topic_name = "camera_sync_result",
                            .imu_topic_name = "bmi088_gyro",
                            .trigger_period_us = 50000,
                            .camera_sync_command_topic_name = "camera_sync_command"})
      : camera_sync_pin_(camera_pin),
        imu_topic_(LibXR::Topic::CreateTopic<ImuSample>(param.imu_topic_name)),
        command_topic_(
            LibXR::Topic::CreateTopic<SyncCommand>(param.camera_sync_command_topic_name)),
        camera_sync_topic_(
            LibXR::Topic::CreateTopic<SyncEvent>(param.camera_sync_topic_name)),
        state_machine_(param.trigger_period_us)
  {
    ASSERT(param.trigger_period_us != 0);

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
  }

 private:
  /// 处理上位机命令：状态机更新后在锁外执行动作。
  /// Handle a host command; the actions run after the state lock is released.
  void OnCommand(bool in_isr, const SyncCommand& command)
  {
    CameraSyncDetail::SyncActions actions;
    {
      LibXR::Mutex::LockGuard lock(state_machine_mutex_);
      actions = state_machine_.OnCommand(command);
    }
    ApplyActions(actions, in_isr);
  }

  /// 以 IMU timestamp 推进状态机：处理触发周期、命令生效与脉冲复位。
  /// Advance the state machine with the IMU timestamp: trigger period, command
  /// application and pulse reset.
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
