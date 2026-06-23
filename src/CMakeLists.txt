execute_process(COMMAND gcc -dumpmachine
    OUTPUT_VARIABLE LINUX_ARCH_FULL
    OUTPUT_STRIP_TRAILING_WHITESPACE)

string(REGEX MATCH "^[^-]+" LINUX_ARCH ${LINUX_ARCH_FULL})
set(DS_VERSION 8.0)
set(CUDA_VER 13.0)

message("-- DeepStream Version: ${DS_VERSION}")
message("-- CUDA Version: ${CUDA_VER}")
message("-- COMPILE_TO_ARCH: ${LINUX_ARCH}")

if("${LINUX_ARCH}" STREQUAL "aarch64")
    message("-- aarch64 Jetson TEGRA")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPLATFORM_TEGRA")
elseif("${LINUX_ARCH}" STREQUAL "x86_64")
    message("-- X86 dGPU")
endif()

set(SRC_PATH /opt/nvidia/deepstream/deepstream-${DS_VERSION})
set(SRC_INC2 ${SRC_PATH}/sources/includes)

set(CANDLESDK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CANDLESDK_BUILD_CANDLETOOL OFF CACHE BOOL "" FORCE)
set(CANDLE_BUILD_STATIC ON CACHE BOOL "" FORCE)

add_subdirectory(../libs/CANdle-SDK ${CMAKE_BINARY_DIR}/candlesdk EXCLUDE_FROM_ALL)

set(COMMON_LIBS
    -lnvdsgst_meta -lnvds_meta -lnvds_yml_parser -lnvdsgst_helper
    -lgstreamer-1.0 -lglib-2.0 -lgobject-2.0
    -lcudart -lcuda
    -lpthread
    candle
)

set(INFERENCE_SOURCES
    main.cpp
    pipeline/inference_mono.cpp
    pipeline/inference_rgb.cpp
    tracking/aim_control.cpp
    comm/serial.cpp
)

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
elseif(NOT CLANG_TIDY)
  message("-- clang-tidy: NOT FOUND (install via 'apt install clang-tidy' or 'brew install llvm')")
endif()

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

target_link_libraries(inference ${COMMON_LIBS})
