#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: MCU-side camera trigger synchronizer driven by timestamped IMU messages
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_result"
  - imu_topic_name: "bmi088_gyro"
  - trigger_div: 10
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>
#include <type_traits>

#include "app_framework.hpp"
#include "gpio.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "transform.hpp"

/**
 * @brief 相机同步触发模块。
 * @details 订阅带时间戳的 IMU Topic，按 IMU 样本分频触发相机 GPIO，并用同一个
 *          IMU 时间戳发布同步结果。BMI088 只负责传感器采样和发布时间戳，本模块不再
 *          依赖 BMI088 内部同步回调。
 */
class CameraSync : public LibXR::Application {
 public:
  using ImuSample = Eigen::Matrix<float, 3, 1>;

  enum class Status : uint8_t {
    OK = 0,
    GPIO_MISSING = 1,
  };

  struct SyncEvent {
    uint32_t trigger_sequence = 0;
    uint32_t imu_sequence = 0;
    uint8_t pin_level = 0;
    uint8_t status = static_cast<uint8_t>(Status::OK);
    uint16_t reserved = 0;
  };

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
      : camera_sync_pin_(hw.template Find<LibXR::GPIO>({camera_pin_name})),
        imu_topic_(imu_topic_name, sizeof(ImuSample)),
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

    app.Register(*this);
  }

  void OnMonitor() override {}

 private:
  void OnImuMessage(bool in_isr, LibXR::MicrosecondTimestamp imu_timestamp) {
    imu_sequence_++;

    if (imu_sequence_ % trigger_div_ != 0) {
      return;
    }

    const bool has_gpio = CameraTrig();
    SyncEvent event;
    event.trigger_sequence = ++trigger_sequence_;
    event.imu_sequence = imu_sequence_;
    event.pin_level = camera_sync_state_ ? 1 : 0;
    event.status = static_cast<uint8_t>(has_gpio ? Status::OK
                                                 : Status::GPIO_MISSING);

    camera_sync_topic_.PublishFromCallback(event, imu_timestamp, in_isr);
  }

  bool CameraTrig() {
    camera_sync_state_ = !camera_sync_state_;

    if (camera_sync_pin_) {
      camera_sync_pin_->Write(camera_sync_state_);
      return true;
    }

    return false;
  }

  LibXR::GPIO* camera_sync_pin_ = nullptr;
  bool camera_sync_state_ = false;

  LibXR::Topic imu_topic_;
  LibXR::Topic camera_sync_topic_;
  LibXR::Topic::Callback imu_callback_;

  uint32_t trigger_div_ = 1;
  uint32_t imu_sequence_ = 0;
  uint32_t trigger_sequence_ = 0;
};
