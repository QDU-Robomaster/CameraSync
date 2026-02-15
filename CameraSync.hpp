#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_euler"
  - ahrs_topic_name: "ahrs_quaternion"
  - bmi088: '@&bmi088'
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "BMI088.hpp"
#include "app_framework.hpp"
#include "libxr_cb.hpp"

/**
 * @brief 相机同步触发模块
 * @details 监听 BMI088 同步信号并在回调中发布姿态数据，同时翻转同步引脚。
 */
class CameraSync : public LibXR::Application {
 public:
  /**
   * @brief 构造 CameraSync 模块
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param camera_pin_name 相机同步引脚名
   * @param camera_sync_topic_name 对外发布同步姿态的 Topic 名称
   * @param ahrs_topic_name 姿态源 Topic 名称
   * @param bmi088 BMI088 模块指针
   */
  CameraSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* camera_pin_name, const char* camera_sync_topic_name,
             const char* ahrs_topic_name, BMI088* bmi088)
      : camera_sync_pin_(hw.template Find<LibXR::GPIO>({camera_pin_name})),
        camera_sync_euler(camera_sync_topic_name,
                          sizeof(LibXR::Quaternion<float>)),
        bmi088_(bmi088),
        ahrs_topic_(ahrs_topic_name, sizeof(LibXR::Quaternion<float>)),
        latest_ahrs_data_(0.0f) {
    if (camera_sync_pin_) {
      camera_sync_pin_->SetConfig(
          {.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
           .pull = LibXR::GPIO::Pull::NONE});
      camera_sync_pin_->Write(false);
    }

    ahrs_callback_ = LibXR::Topic::Callback::Create(
        [](bool, float* arg, LibXR::RawData& data) {
          *arg = *reinterpret_cast<float*>(data.addr_);
        },
        &latest_ahrs_data_);
    ahrs_topic_.RegisterCallback(ahrs_callback_);

    if (bmi088_) {
      bmi088_->RegisterSyncCallback(GetSyncCallback());
    }
  }

  /**
   * @brief 获取同步回调
   * @return LibXR::Callback<> 回调对象
   */
  LibXR::Callback<> GetSyncCallback() {
    return LibXR::Callback<>::Create(
        [](bool in_isr, CameraSync* self) {
          if (self->bmi088_ && self->bmi088_->GetsyncStatus()) {
            self->bmi088_->SetsyncStatus(false);

            self->ahrs_topic_.PublishFromCallback(
                &self->latest_ahrs_data_, sizeof(self->latest_ahrs_data_),
                in_isr);
            self->camera_sync_euler.PublishFromCallback(
                &self->latest_ahrs_data_, sizeof(self->latest_ahrs_data_),
                in_isr);
            self->CameraTrig();
          }
        },
        this);
  }

  /**
   * @brief 监控回调
   */
  void OnMonitor() override {};

 private:
  LibXR::GPIO* camera_sync_pin_ = nullptr;
  bool camera_sync_state_ = false;
  LibXR::Topic camera_sync_euler;
  BMI088* bmi088_ = nullptr;

  LibXR::Topic ahrs_topic_;
  float latest_ahrs_data_;
  LibXR::Callback<LibXR::RawData&> ahrs_callback_;

  void CameraTrig() {
    if (camera_sync_pin_) {
      camera_sync_state_ = !camera_sync_state_;
      camera_sync_pin_->Write(camera_sync_state_);
    }
  }
};
