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

foreach(nvds_lib nvdsgst_meta nvds_meta nvds_yml_parser nvdsgst_helper nvbufsurface)
  check_ds_lib(${nvds_lib})
endforeach()

# EXCLUDE_FROM_ALL on both: only what our targets link against gets built.
FetchContent_MakeAvailable(candlesdk kalman_cpp)

add_subdirectory(src)
