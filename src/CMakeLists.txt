# Target architecture from the compiler triple (aarch64 = Jetson, x86_64 = dGPU)
execute_process(
  COMMAND gcc -dumpmachine
  OUTPUT_VARIABLE LINUX_ARCH_FULL
  OUTPUT_STRIP_TRAILING_WHITESPACE)

string(REGEX MATCH "^[^-]+" LINUX_ARCH ${LINUX_ARCH_FULL})
set(DS_VERSION 8.0)
set(CUDA_VER 13.0)

message("-- DeepStream Version: ${DS_VERSION}")
message("-- CUDA Version: ${CUDA_VER}")
message("-- Architecture: ${LINUX_ARCH}")

if("${LINUX_ARCH}" STREQUAL "aarch64")
  message("-- aarch64 Jetson TEGRA")
elseif("${LINUX_ARCH}" STREQUAL "x86_64")
  message("-- X86 dGPU")
endif()

set(SRC_PATH /opt/nvidia/deepstream/deepstream-${DS_VERSION})
set(SRC_INC2 ${SRC_PATH}/sources/includes)

# clang-tidy integration Override with -DCMAKE_CXX_CLANG_TIDY=OFF to disable
find_program(CLANG_TIDY NAMES clang-tidy)
if(CLANG_TIDY
   AND NOT CMAKE_CXX_CLANG_TIDY STREQUAL "OFF"
   AND NOT CMAKE_CXX_CLANG_TIDY STREQUAL "")
  set(CMAKE_CXX_CLANG_TIDY
      "${CLANG_TIDY}" "--extra-arg=-std=c++17"
      "--extra-arg=-I${CMAKE_CURRENT_SOURCE_DIR}" "--warnings-as-errors=*")
  message("-- clang-tidy: ${CLANG_TIDY}")
else()
  unset(CMAKE_CXX_CLANG_TIDY CACHE)
  set(CMAKE_CXX_CLANG_TIDY "")
  if(NOT CLANG_TIDY)
    message(
      "-- clang-tidy: NOT FOUND (install via 'apt install clang-tidy' or 'brew install llvm')"
    )
  endif()
endif()

set(INFERENCE_SOURCES
    main.cpp
    core/config.cpp
    pipeline/inference.cpp
    tracking/aim_control.cpp
    tracking/target_filter.cpp
    tracking/prediction_overlay.cpp
    comm/candle_motor.cpp
    comm/command_listener.cpp
    comm/sensor_module.cpp)

set(INFERENCE_HEADERS
    core/config.h
    core/state.h
    core/types.h
    pipeline/inference_mono.h
    pipeline/inference_rgb.h
    tracking/aim_control.h
    tracking/prediction_overlay.h
    tracking/target_filter.h
    comm/candle_motor.h
    comm/command_listener.h
    comm/sensor_module.h)

add_executable(inference ${INFERENCE_SOURCES})
target_sources(inference PRIVATE ${INFERENCE_HEADERS})

target_include_directories(
  inference
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
          ${SRC_INC2}
          /usr/local/cuda-${CUDA_VER}/include
          /usr/include/gstreamer-1.0
          /usr/include/glib-2.0
          /usr/lib/${LINUX_ARCH}-linux-gnu/glib-2.0/include)

target_link_directories(
  inference PRIVATE ${SRC_PATH}/lib /usr/local/cuda-${CUDA_VER}/lib64
  /usr/lib/${LINUX_ARCH}-linux-gnu
  /usr/lib/${LINUX_ARCH}-linux-gnu/gstreamer-1.0)

target_link_libraries(
  inference
  PRIVATE nvdsgst_meta
          nvds_meta
          nvds_yml_parser
          nvdsgst_helper
          nvbufsurface
          gstreamer-1.0
          glib-2.0
          gobject-2.0
          cudart
          cuda
          pthread
          candle
          kalman::kalman)
