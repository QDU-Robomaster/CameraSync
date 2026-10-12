# CameraSync 模块 CMake 配置：仅头文件。
# CameraSync Module CMake configuration: headers only.

target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

get_filename_component(_MODULE_DIRNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
set(_DEPS_TARGET "${_MODULE_DIRNAME}_deps")
add_library(${_DEPS_TARGET} INTERFACE)
set_property(GLOBAL APPEND PROPERTY XR_MODULE_DEPS ${_DEPS_TARGET})

# 状态机测试只在主机构建；MCU 交叉编译时跳过。
# The state-machine test builds on the host only; cross builds for the MCU skip it.
if(BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tests
                   ${CMAKE_CURRENT_BINARY_DIR}/CameraSync_tests)
endif()
