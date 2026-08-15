cmake_minimum_required(VERSION 3.16)
project(CameraSyncStateMachineTests LANGUAGES CXX)

include(CTest)

add_executable(camera_sync_state_machine_test
               camera_sync_state_machine_test.cpp)
target_include_directories(camera_sync_state_machine_test
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_compile_features(camera_sync_state_machine_test PRIVATE cxx_std_20)

if(MSVC)
  target_compile_options(camera_sync_state_machine_test PRIVATE /utf-8 /W4 /WX)
else()
  target_compile_options(camera_sync_state_machine_test PRIVATE
                         -Wall -Wextra -Wpedantic -Werror)
endif()

add_test(NAME camera_sync_state_machine_test
         COMMAND camera_sync_state_machine_test)
