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

# clang-tidy integration
# Override with -DCMAKE_CXX_CLANG_TIDY=OFF to disable
find_program(CLANG_TIDY NAMES clang-tidy)
if(CLANG_TIDY AND NOT CMAKE_CXX_CLANG_TIDY STREQUAL "OFF")
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

add_subdirectory(../deps/CANdle-SDK ${CMAKE_BINARY_DIR}/candlesdk EXCLUDE_FROM_ALL)

set(INFERENCE_SOURCES
    main.cpp
    core/config.cpp
    pipeline/inference.cpp
    tracking/aim_control.cpp
    tracking/target_filter.cpp
    comm/candle_motor.cpp
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
    nv_meta
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
