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
 * @details 模块以 IMU Topic 消息的时间戳为时钟，按周期翻转一个 GPIO 触发相机，每个
 *          触发边沿发布一个 FRAME_TRIGGER，其时间戳为产生该边沿的 IMU 消息时间戳。上位机
 *          的 STOP_TRIGGER 与 START_TRIGGER 在下一条 IMU 消息处生效，并回复 ACK。时序与
 *          命令规则见 CameraSyncDetail::StateMachine。
 *          The Module uses the timestamps of the IMU Topic messages as its clock and
 *          toggles one GPIO every period to trigger the camera; each trigger edge
 *          publishes a FRAME_TRIGGER stamped with the time of the IMU message that
 *          produced it. STOP_TRIGGER and START_TRIGGER from the host take effect at the
 *          next IMU message and are answered with an ACK. See
 *          CameraSyncDetail::StateMachine for the timing and command rules.
 *
 *          两个订阅回调在发布者的线程中同步执行，状态机由一个 Mutex 串行访问，所以 IMU
 *          与命令 Topic 须在线程上下文中发布。GPIO 写入与事件发布在释放锁之后执行。
 *          Both subscription callbacks run synchronously on the publisher's thread and
 *          the state machine is serialised by a Mutex, so the IMU and command Topics
 *          are published from thread context. GPIO writes and event publication run
 *          after the lock is released.
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
   * @details 构造时写的低电平与有效电平无关；第一条 STOP_TRIGGER 或
   *          START_TRIGGER 生效时，按命令中的有效电平写无效电平。上电后即以
   *          `trigger_period_us` 运行。Topic 回调注册后不注销，实例与程序同寿命。
   *          The low level written here does not depend on the active level; the first
   *          STOP_TRIGGER or START_TRIGGER to apply writes the inactive level of the
   *          active level it carries. After power-up the Module runs at
   *          `trigger_period_us`. The Topic callbacks stay registered, so the instance
   *          lives as long as the program.
   *
   * @param camera_pin 连接相机硬件触发输入的 GPIO。
   *                   GPIO wired to the hardware trigger input of the camera.
   * @param param 配置参数；trigger_period_us 为 0 时 REQUIRE 失败（Release 下同样检查）。
   *              Configuration parameters; REQUIRE fails when trigger_period_us is 0,
   *              in Release builds as well.
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
    REQUIRE(param.trigger_period_us != 0);

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

  /// 以 IMU 消息时间戳推进状态机：结束上一个脉冲、执行待生效命令、按周期输出边沿。
  /// Advance the state machine with the IMU message time: end the previous pulse,
  /// apply a pending command, and emit an edge when a period is reached.
  void OnImuMessage(bool in_isr, LibXR::MicrosecondTimestamp imu_timestamp)
  {
    CameraSyncDetail::SyncActions actions;
    {
      LibXR::Mutex::LockGuard lock(state_machine_mutex_);
      actions = state_machine_.OnImu(static_cast<uint64_t>(imu_timestamp));
    }
    ApplyActions(actions, in_isr);
  }

  /// 按状态机给出的顺序写 GPIO、发布事件；事件时间戳取自状态机（ACK 重发时为原时间戳）。
  /// Write the GPIO and publish the events in the order the state machine gave; the
  /// event timestamps come from the state machine (the original one for a replayed ACK).
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

  LibXR::GPIO& camera_sync_pin_;  ///< 相机触发输入 / Camera trigger input

  LibXR::Topic imu_topic_;                   ///< 时间基准（订阅）/ Time base (subscribed)
  LibXR::Topic command_topic_;               ///< 上位机命令（订阅）/ Host commands
  LibXR::Topic camera_sync_topic_;           ///< ACK 与边沿（发布）/ ACKs and edges
  LibXR::Topic::Callback imu_callback_;      ///< IMU 订阅回调 / IMU callback
  LibXR::Topic::Callback command_callback_;  ///< 命令订阅回调 / Command callback

  LibXR::Mutex state_machine_mutex_{};  ///< 串行化两个回调 / Serialises both callbacks
  CameraSyncDetail::StateMachine state_machine_;  ///< 触发逻辑 / Trigger logic
};
