cmake_minimum_required(VERSION 3.28)
project(
  patronus-turret
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

# NVIDIA Jetson AGX Thor (Blackwell, sm_11.0) Use 'native' for auto-detection,
# or set explicitly for cross-compilation: -DCMAKE_CUDA_ARCHITECTURES=110
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
  GIT_REPOSITORY git@github.com:ioeftedal/kalman-cpp.git
  GIT_TAG 05ec5881d3d516c6e0d1de8ebad4fdd28bdf936a
  EXCLUDE_FROM_ALL)
set(KALMAN_BUILD_TESTS
    OFF
    CACHE BOOL "" FORCE)
set(KALMAN_INSTALL
    OFF
    CACHE BOOL "" FORCE)

# EXCLUDE_FROM_ALL on both: only what our targets link against gets built.
FetchContent_MakeAvailable(candlesdk kalman_cpp)

add_subdirectory(src)
