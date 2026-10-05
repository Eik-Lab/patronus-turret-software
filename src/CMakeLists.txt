# Target architecture from the compiler triple (aarch64 = Jetson, x86_64 = dGPU)
execute_process(COMMAND gcc -dumpmachine
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

set(CANDLESDK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CANDLESDK_BUILD_CANDLETOOL OFF CACHE BOOL "" FORCE)
set(CANDLE_BUILD_STATIC ON CACHE BOOL "" FORCE)

# CANdle-SDK first, deliberately. CMAKE_CXX_CLANG_TIDY is set below and applies to
# every target created after it, so pulling in third-party code afterwards would
# lint that code against our style rules -- it uses `fileRaw`/`resetClr` and
# trips --warnings-as-errors, which fails the build for reasons unrelated to us.
add_subdirectory(../deps/CANdle-SDK ${CMAKE_BINARY_DIR}/candlesdk EXCLUDE_FROM_ALL)

# clang-tidy integration
# Override with -DCMAKE_CXX_CLANG_TIDY=OFF to disable
find_program(CLANG_TIDY NAMES clang-tidy)
# Empty as well as "OFF": the top level normalises `-DCMAKE_CXX_CLANG_TIDY=OFF` to
# the empty string before we get here, and re-enabling it would ignore the flag.
if(CLANG_TIDY AND NOT CMAKE_CXX_CLANG_TIDY STREQUAL "OFF" AND NOT CMAKE_CXX_CLANG_TIDY STREQUAL "")
  set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY}"
    "--extra-arg=-std=c++17"
    "--extra-arg=-I${PROJECT_SOURCE_DIR}/include"
    "--warnings-as-errors=*"
  )
  message("-- clang-tidy: ${CLANG_TIDY}")
else()
  unset(CMAKE_CXX_CLANG_TIDY CACHE)
  set(CMAKE_CXX_CLANG_TIDY "")
  if(NOT CLANG_TIDY)
    message("-- clang-tidy: NOT FOUND (install via 'apt install clang-tidy' or 'brew install llvm')")
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
    comm/sensor_module.cpp
    tools/motor_test.cpp
    tools/calibrate_focal.cpp
)

add_executable(inference ${INFERENCE_SOURCES})

target_include_directories(inference PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${SRC_INC2}
    /usr/local/cuda-${CUDA_VER}/include
    /usr/include/gstreamer-1.0
    /usr/include/glib-2.0
    /usr/lib/${LINUX_ARCH}-linux-gnu/glib-2.0/include
)

target_link_directories(inference PRIVATE
    ${SRC_PATH}/lib
    /usr/local/cuda-${CUDA_VER}/lib64
    /usr/lib/${LINUX_ARCH}-linux-gnu
    /usr/lib/${LINUX_ARCH}-linux-gnu/gstreamer-1.0
)

target_link_libraries(inference
    nvdsgst_meta
    nvds_meta
    nvds_yml_parser
    nvdsgst_helper
    gstreamer-1.0
    glib-2.0
    gobject-2.0
    cudart
    cuda
    pthread
    candle
    kalman
)

# --- Home calibration tool ---
add_executable(calibrate_home
    tools/calibrate_home.cpp
    core/config.cpp
    comm/candle_motor.cpp
)

target_include_directories(calibrate_home PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    /usr/include/glib-2.0
    /usr/lib/${LINUX_ARCH}-linux-gnu/glib-2.0/include
)

target_link_directories(calibrate_home PRIVATE
    /usr/lib/${LINUX_ARCH}-linux-gnu
)

target_link_libraries(calibrate_home
    candle
    glib-2.0
    pthread
)

# --- Target filter tests ---
# Deliberately standalone rather than a GTest target: the filter is the one piece
# of this system whose correctness is not observable without cameras and a CAN
# bus, so it needs its own executable that runs anywhere. No DeepStream, no
# CANdle, no glib -- just the filter and the IMM.
add_executable(test_target_filter
    ../tests/test_target_filter.cpp
    tracking/target_filter.cpp
)

target_include_directories(test_target_filter PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(test_target_filter PRIVATE kalman)

add_test(NAME target_filter COMMAND test_target_filter)
