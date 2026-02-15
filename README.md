# CameraSync

## 1. 模块作用
相机同步模块。将姿态采样与相机触发时序对齐。

## 2. 主要函数说明
1. GetSyncCallback: 提供 IMU 同步回调入口。
2. CameraTrig: 翻转同步引脚并发布同步姿态。
3. OnMonitor: 监控钩子（当前为空实现）。

## 3. 接入步骤
1. 添加模块并配置 camera_pin_name、camera_sync_topic_name、ahrs_topic_name。
2. 把 BMI088 同步回调绑定到本模块。
3. 检查同步引脚时序和 Topic 数据是否一致。

标准命令流程：
    xrobot_add_mod CameraSync --instance-id camerasync
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: CameraSync
entry_header: Modules/CameraSync/CameraSync.hpp
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_euler"
  - ahrs_topic_name: "ahrs_quaternion"
  - bmi088: '@&bmi088'
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
[]

Depends:
[]

## 6. 代码入口
Modules/CameraSync/CameraSync.hpp
