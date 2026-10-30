cmake_minimum_required(VERSION 3.28)
project(
  patronus-bird-tracker
  VERSION 1.0.0
  LANGUAGES C CXX)

if(CMAKE_CXX_CLANG_TIDY STREQUAL "OFF")
  unset(CMAKE_CXX_CLANG_TIDY CACHE)
  set(CMAKE_CXX_CLANG_TIDY "")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

option(PATRONUS_REPLAY_ONLY "Build hardware-independent tracking replay only"
       OFF)

# Jetson AGX Thor is Blackwell, sm_11.0. Override with
# -DCMAKE_CUDA_ARCHITECTURES=native to build for the local GPU instead.
if(CMAKE_CUDA_COMPILER)
  set(CMAKE_CUDA_ARCHITECTURES
      110
      CACHE STRING "CUDA architectures (110 = Blackwell sm_11.0)")
endif()

# Third-party dependencies, fetched at configure time
include(FetchContent)

FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c # 3.4.0
  OVERRIDE_FIND_PACKAGE EXCLUDE_FROM_ALL)
set(BUILD_TESTING
    OFF
    CACHE BOOL "Build dependency tests")
set(EIGEN_BUILD_DOC
    OFF
    CACHE BOOL "Build Eigen documentation")
FetchContent_MakeAvailable(Eigen3)

FetchContent_Declare(
  candlesdk
  GIT_REPOSITORY https://codeberg.org/fatum/CANdle-SDK-CPP17.git
  GIT_TAG 60c4e2c973de5632d81783271d54941a0202b91b
  GIT_SUBMODULES commons/googletest EXCLUDE_FROM_ALL)
set(CANDLESDK_BUILD_EXAMPLES
    OFF
    CACHE BOOL "" FORCE)
set(CANDLESDK_BUILD_CANDLETOOL
    OFF
    CACHE BOOL "" FORCE)
set(CANDLE_BUILD_STATIC
    ON
    CACHE BOOL "" FORCE)

FetchContent_Declare(
  kalman_cpp
  GIT_REPOSITORY https://github.com/ioeftedal/kalman-cpp.git
  GIT_TAG d7c6fd2ce4b8b9b683c5c76ac8bb4ce5da4bcdef
  EXCLUDE_FROM_ALL)
set(KALMAN_BUILD_TESTS
    OFF
    CACHE BOOL "" FORCE)
set(KALMAN_INSTALL
    OFF
    CACHE BOOL "" FORCE)

if(NOT TARGET Eigen3::Eigen AND NOT Eigen3_FOUND)
  find_package(Eigen3 3.4 QUIET NO_MODULE)
endif()
if(NOT Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
  message(
    FATAL_ERROR
      "Eigen3 >= 3.4 not found — required by kalman-cpp.\n"
      "  nix develop                       # the flake devShell provides eigen\n"
      "  sudo apt-get install libeigen3-dev\n"
      "  -DEigen3_DIR=/path/to/eigen/share/eigen3/cmake")
endif()

if(PATRONUS_REPLAY_ONLY)
  FetchContent_MakeAvailable(kalman_cpp)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GLIB REQUIRED IMPORTED_TARGET glib-2.0)
  add_executable(
    tracking_replay
    src/tools/tracking_replay.cpp src/core/config.cpp
    src/tracking/target_filter.cpp src/tracking/aim_control.cpp
    src/tracking/control_state.cpp)
  target_include_directories(tracking_replay PRIVATE src)
  target_link_libraries(tracking_replay PRIVATE kalman::kalman PkgConfig::GLIB)
  find_package(Threads REQUIRED)
  add_executable(telemetry_fixture tools/test_telemetry.cpp
                                   src/tracking/telemetry.cpp)
  target_include_directories(telemetry_fixture PRIVATE src)
  target_link_libraries(telemetry_fixture PRIVATE Threads::Threads
                                                  kalman::kalman)
  enable_testing()
  add_executable(observation_channel_test tools/test_observation_channel.cpp)
  target_include_directories(observation_channel_test PRIVATE src)
  target_link_libraries(observation_channel_test PRIVATE Threads::Threads)
  add_test(NAME observation_channel COMMAND observation_channel_test)
  add_executable(motor_result_test tools/test_motor_result.cpp
                                   src/tracking/control_state.cpp)
  target_include_directories(motor_result_test PRIVATE src)
  add_test(NAME motor_result COMMAND motor_result_test)
  add_executable(manual_control_test tools/test_manual_control.cpp
                                     src/tracking/control_state.cpp)
  target_include_directories(manual_control_test PRIVATE src)
  add_test(NAME manual_control COMMAND manual_control_test)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(actuator_calibration src/tools/actuator_calibration.cpp)
  target_compile_definitions(actuator_calibration
                             PRIVATE PATRONUS_CALIBRATION_SIMULATION=1)
  target_link_libraries(actuator_calibration PRIVATE PkgConfig::GLIB
                                                     Threads::Threads)
  add_test(
    NAME actuator_calibration
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/test_actuator_calibration.py
      $<TARGET_FILE:actuator_calibration>)
  add_test(
    NAME tracking_replay
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/test_tracking_replay.py
      $<TARGET_FILE:tracking_replay> $<TARGET_FILE:telemetry_fixture>)
  pkg_check_modules(GSTREAMER QUIET IMPORTED_TARGET gstreamer-1.0)
  if(GSTREAMER_FOUND)
    add_executable(run_trace_fixture tools/test_run_trace.cpp
                                     src/pipeline/run_trace.cpp)
    target_include_directories(run_trace_fixture PRIVATE src)
    target_link_libraries(run_trace_fixture PRIVATE PkgConfig::GSTREAMER
                                                    Threads::Threads)
    add_test(
      NAME live_recording
      COMMAND
        ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/test_live_run.py --trace-driver
        $<TARGET_FILE:run_trace_fixture>)
  else()
    add_test(NAME live_recording
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/tools/test_live_run.py)
  endif()
  return()
endif()

set(DS_VERSION 8.0)
set(DS_PATH "/opt/nvidia/deepstream/deepstream-${DS_VERSION}")
if(NOT EXISTS "${DS_PATH}")
  message(FATAL_ERROR "DeepStream ${DS_VERSION} not found at ${DS_PATH}")
endif()

function(check_ds_lib libname)
  set(libpath "${DS_PATH}/lib/lib${libname}.so")
  if(NOT EXISTS ${libpath})
    message(FATAL_ERROR "Missing lib${libname}.so at ${libpath}\n"
                        "Please make sure DeepStream is properly installed")
  endif()
endfunction()

foreach(nvds_lib nvdsgst_meta nvds_meta nvds_yml_parser nvdsgst_helper
                 nvbufsurface)
  check_ds_lib(${nvds_lib})
endforeach()

# EXCLUDE_FROM_ALL on both: only what our targets link against gets built.
FetchContent_MakeAvailable(candlesdk kalman_cpp)

add_subdirectory(src)
