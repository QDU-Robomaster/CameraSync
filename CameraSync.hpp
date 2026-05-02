#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 由带时间戳 IMU 消息驱动的 MCU 侧相机触发同步模块
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_result"
  - imu_topic_name: "bmi088_gyro"
  - trigger_div: 10
  - camera_sync_command_topic_name: "camera_sync_command"
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>
#include <limits>

#include "app_framework.hpp"
#include "gpio.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "transform.hpp"

/**
 * @brief 相机同步触发模块。
 * @details 模块只把 IMU topic 的 envelope timestamp 当作时间基准。正常状态下，
 *          每 trigger_div 个 IMU 样本翻转一次相机 GPIO。上位机命令只描述一次同步
 *          标记：在下一个反向电平周期开始执行，把这段反向周期拉长 div 倍，结束后
 *          第一个有效电平边沿就是同步点。SyncEvent 只回传 seq；消息 envelope
 *          timestamp 就是该同步点实际对齐的 IMU 传感器时间。
 */
class CameraSync : public LibXR::Application {
 public:
  using ImuSample = Eigen::Matrix<float, 3, 1>;

  /**
   * @brief 上位机同步命令。
   * @details div 是单次反向电平周期拉长倍率，必须大于 0；active_level 是相机的
   *          有效触发电平，0 表示低有效，非 0 表示高有效；seq 由上位机每次同步
   *          自增，MCU 在对应同步点回传同一个 seq。
   */
  struct SyncCommand {
    uint32_t div = 1;
    uint32_t active_level = 1;
    uint32_t seq = 0;
  };

  /**
   * @brief 同步点回执。
   * @details 只回传 seq。实际同步时间使用 topic 消息自带 timestamp，不再复制到
   *          payload 里。
   */
  struct SyncEvent {
    uint32_t seq = 0;
  };

  /**
   * @brief 构造 CameraSync 模块。
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param camera_pin_name 相机触发 GPIO 名称
   * @param camera_sync_topic_name 同步结果 Topic 名称
   * @param imu_topic_name 作为同步基准的 IMU Topic 名称
   * @param trigger_div 每多少个 IMU 样本触发一次相机，必须大于 0
   * @param camera_sync_command_topic_name 上位机同步命令 Topic 名称
   */
  CameraSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* camera_pin_name, const char* camera_sync_topic_name,
             const char* imu_topic_name, uint32_t trigger_div,
             const char* camera_sync_command_topic_name)
      : camera_sync_pin_(hw.template Find<LibXR::GPIO>({camera_pin_name})),
        imu_topic_(imu_topic_name, sizeof(ImuSample)),
        command_topic_(camera_sync_command_topic_name, sizeof(SyncCommand)),
        camera_sync_topic_(camera_sync_topic_name, sizeof(SyncEvent)),
        trigger_div_(trigger_div),
        period_ticks_(trigger_div) {
    ASSERT(trigger_div_ != 0);

    if (camera_sync_pin_) {
      camera_sync_pin_->SetConfig(
          {.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
           .pull = LibXR::GPIO::Pull::NONE});
      camera_sync_pin_->Write(false);
    }

    imu_callback_ = LibXR::Topic::Callback::Create(
        [](bool in_isr, CameraSync* self, LibXR::MicrosecondTimestamp timestamp,
           const ImuSample&) { self->OnImuMessage(in_isr, timestamp); },
        this);
    imu_topic_.RegisterCallback(imu_callback_);

    command_callback_ = LibXR::Topic::Callback::Create(
        [](bool, CameraSync* self, LibXR::MicrosecondTimestamp,
           const SyncCommand& command) { self->OnCommand(command); },
        this);
    command_topic_.RegisterCallback(command_callback_);

    app.Register(*this);
  }

  void OnMonitor() override {}

 private:
  enum class SyncState : uint8_t {
    NORMAL = 0,
    WAIT_REVERSE_PERIOD = 1,
    STRETCH_REVERSE_PERIOD = 2,
  };

  void OnCommand(const SyncCommand& command) {
    // 上位机应等待回执后再发下一条命令；模块只保留最新一条待执行命令。
    ASSERT(command.div != 0);
    ASSERT(command.div <= std::numeric_limits<uint32_t>::max() / trigger_div_);

    pending_command_data_.div = command.div;
    pending_command_data_.active_level = command.active_level == 0 ? 0U : 1U;
    pending_command_data_.seq = command.seq;
    pending_command_ = true;
  }

  void TryStartPendingCommand() {
    if (sync_state_ != SyncState::NORMAL || !pending_command_) {
      return;
    }

    pending_command_ = false;
    active_command_ = pending_command_data_;
    sync_state_ = SyncState::WAIT_REVERSE_PERIOD;
  }

  void OnImuMessage(bool in_isr, LibXR::MicrosecondTimestamp imu_timestamp) {
    TryStartPendingCommand();
    ticks_since_trigger_++;

    if (ticks_since_trigger_ < period_ticks_) {
      return;
    }

    ticks_since_trigger_ = 0;
    const uint32_t pin_level = CameraTrig();

    if (sync_state_ == SyncState::WAIT_REVERSE_PERIOD &&
        pin_level != active_command_.active_level) {
      sync_state_ = SyncState::STRETCH_REVERSE_PERIOD;
      period_ticks_ = trigger_div_ * active_command_.div;
      return;
    }

    if (sync_state_ == SyncState::STRETCH_REVERSE_PERIOD) {
      if (pin_level == active_command_.active_level) {
        SyncEvent event;
        event.seq = active_command_.seq;
        camera_sync_topic_.PublishFromCallback(event, imu_timestamp, in_isr);
      }

      sync_state_ = SyncState::NORMAL;
      period_ticks_ = trigger_div_;
      active_command_ = {};
      TryStartPendingCommand();
    }
  }

  uint32_t CameraTrig() {
    camera_sync_state_ = !camera_sync_state_;

    if (camera_sync_pin_) {
      camera_sync_pin_->Write(camera_sync_state_);
    }

    return camera_sync_state_ ? 1 : 0;
  }

  LibXR::GPIO* camera_sync_pin_ = nullptr;
  bool camera_sync_state_ = false;

  LibXR::Topic imu_topic_;
  LibXR::Topic command_topic_;
  LibXR::Topic camera_sync_topic_;
  LibXR::Topic::Callback imu_callback_;
  LibXR::Topic::Callback command_callback_;

  uint32_t trigger_div_ = 1;
  // 当前 GPIO 翻转周期，正常为 trigger_div_，同步标记反向周期内临时拉长。
  uint32_t period_ticks_ = 1;
  uint32_t ticks_since_trigger_ = 0;

  bool pending_command_ = false;
  SyncCommand pending_command_data_;

  SyncState sync_state_ = SyncState::NORMAL;
  SyncCommand active_command_;
};
