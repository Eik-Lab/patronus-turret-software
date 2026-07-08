cmake_minimum_required(VERSION 3.10)
project(inference)

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
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPLATFORM_TEGRA")
    # If you want to use C++ on Jetson, rename the source file from .c to .cpp
    # set(CMAKE_CXX_STANDARD 11)
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPLATFORM_TEGRA")

elseif("${LINUX_ARCH}" STREQUAL "x86_64")
    message("-- X86 dGPU")
    set(CMAKE_CXX_STANDARD 17)
    # set(CMAKE_C_STANDARD 11)

endif()

set(SRC_PATH /opt/nvidia/deepstream/deepstream-${DS_VERSION})
set(SRC_INC2 ${SRC_PATH}/sources/includes)

include_directories(
        ${PROJECT_SOURCE_DIR}/deepstream/nvdinfer
        ${PROJECT_SOURCE_DIR}
        ${SRC_INC2}
        /usr/local/cuda-${CUDA_VER}/include
        /usr/include/gstreamer-1.0
        /usr/include/glib-2.0
        /usr/lib/${LINUX_ARCH}-linux-gnu/glib-2.0/include
)

link_directories(
        ${SRC_PATH}/lib
        /usr/local/cuda-${CUDA_VER}/lib64
        /usr/lib/${LINUX_ARCH}-linux-gnu
        /usr/lib/${LINUX_ARCH}-linux-gnu/gstreamer-1.0
)

set(SRCS
    deepstream/nvdinfer/yolo_inference_mono.cpp
    deepstream/nvdinfer/yolo_inference_rgb.cpp
    deepstream/nvdinfer/
    main.cpp
)

add_executable(inference ${SRCS})

add_subdirectory(deps/CANdle-SDK)
target_link_libraries(inference PRIVATE candle)

target_link_libraries(inference
        -lnvdsgst_meta -lnvds_meta -lnvds_yml_parser -lnvdsgst_helper
        -lgstreamer-1.0 -lglib-2.0 -lgobject-2.0
        -lcudart -lcuda
        -lpthread)
