#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: MCU-side camera trigger synchronizer driven by timestamped IMU messages
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

#include <atomic>
#include <cstdint>
#include <limits>
#include <type_traits>

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
   * @details div 是单次反向电平周期拉长倍率，0 按 1 处理；active_level 是相机的
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

  static_assert(std::is_trivially_copyable_v<SyncCommand>,
                "CameraSync::SyncCommand must be trivially copyable");
  static_assert(std::is_trivially_copyable_v<SyncEvent>,
                "CameraSync::SyncEvent must be trivially copyable");

  /**
   * @brief 构造 CameraSync 模块。
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param camera_pin_name 相机触发 GPIO 名称
   * @param camera_sync_topic_name 同步结果 Topic 名称
   * @param imu_topic_name 作为同步基准的 IMU Topic 名称
   * @param trigger_div 每多少个 IMU 样本触发一次相机；0 会按 1 处理
   */
  CameraSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* camera_pin_name, const char* camera_sync_topic_name,
             const char* imu_topic_name, uint32_t trigger_div)
      : CameraSync(hw, app, camera_pin_name, camera_sync_topic_name,
                   imu_topic_name, trigger_div, "camera_sync_command") {}

  /**
   * @brief 构造 CameraSync 模块。
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param camera_pin_name 相机触发 GPIO 名称
   * @param camera_sync_topic_name 同步结果 Topic 名称
   * @param imu_topic_name 作为同步基准的 IMU Topic 名称
   * @param trigger_div 每多少个 IMU 样本触发一次相机；0 会按 1 处理
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
        trigger_div_(trigger_div == 0 ? 1 : trigger_div) {
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

  static uint32_t NormalizeDiv(uint32_t div) {
    return div == 0 ? 1 : div;
  }

  uint32_t CurrentPeriodTicks() const {
    const uint32_t div =
        sync_state_ == SyncState::STRETCH_REVERSE_PERIOD ? active_div_ : 1;
    if (div > std::numeric_limits<uint32_t>::max() / trigger_div_) {
      return std::numeric_limits<uint32_t>::max();
    }

    return trigger_div_ * div;
  }

  void OnCommand(const SyncCommand& command) {
    // The host waits for the matching ack before sending the next command, so
    // the module only keeps the latest unconsumed pending command.
    pending_div_.store(NormalizeDiv(command.div), std::memory_order_relaxed);
    pending_active_level_.store(command.active_level == 0 ? 0U : 1U,
                                std::memory_order_relaxed);
    pending_seq_.store(command.seq, std::memory_order_relaxed);
    pending_command_.store(true, std::memory_order_release);
  }

  void TryStartPendingCommand() {
    if (sync_state_ != SyncState::NORMAL ||
        !pending_command_.exchange(false, std::memory_order_acquire)) {
      return;
    }

    active_div_ = pending_div_.load(std::memory_order_relaxed);
    active_level_ = pending_active_level_.load(std::memory_order_relaxed);
    active_seq_ = pending_seq_.load(std::memory_order_relaxed);
    sync_state_ = SyncState::WAIT_REVERSE_PERIOD;
  }

  void OnImuMessage(bool in_isr, LibXR::MicrosecondTimestamp imu_timestamp) {
    TryStartPendingCommand();
    ticks_since_trigger_++;

    if (ticks_since_trigger_ < CurrentPeriodTicks()) {
      return;
    }

    ticks_since_trigger_ = 0;
    const uint32_t pin_level = CameraTrig();

    if (sync_state_ == SyncState::WAIT_REVERSE_PERIOD &&
        pin_level != active_level_) {
      sync_state_ = SyncState::STRETCH_REVERSE_PERIOD;
      return;
    }

    if (sync_state_ == SyncState::STRETCH_REVERSE_PERIOD) {
      if (pin_level == active_level_) {
        SyncEvent event;
        event.seq = active_seq_;
        camera_sync_topic_.PublishFromCallback(event, imu_timestamp, in_isr);
      }

      sync_state_ = SyncState::NORMAL;
      active_div_ = 1;
      active_level_ = 1;
      active_seq_ = 0;
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
  uint32_t ticks_since_trigger_ = 0;

  std::atomic<bool> pending_command_{false};
  std::atomic<uint32_t> pending_div_{1};
  std::atomic<uint32_t> pending_active_level_{1};
  std::atomic<uint32_t> pending_seq_{0};

  SyncState sync_state_ = SyncState::NORMAL;
  uint32_t active_div_ = 1;
  uint32_t active_level_ = 1;
  uint32_t active_seq_ = 0;
};
