cmake_minimum_required(VERSION 3.28)
project(
  patronus-turret
  VERSION 1.0.0
  LANGUAGES C CXX)

# `-DCMAKE_CXX_CLANG_TIDY=OFF` lands in the cache as the literal string "OFF",
# and CMake treats a CMAKE_CXX_CLANG_TIDY it cannot resolve as the *program to
# run* -- so every target it reaches fails with `Error running 'OFF'`. Normalise
# it here, before any subdirectory is added, so the documented way to disable
# linting actually works. Empty means "no tidy".
if(CMAKE_CXX_CLANG_TIDY STREQUAL "OFF")
  unset(CMAKE_CXX_CLANG_TIDY CACHE)
  set(CMAKE_CXX_CLANG_TIDY "")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Must be in the top-level scope for `ctest` to find tests registered in a
# subdirectory.
enable_testing()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# NVIDIA Jetson AGX Thor (Blackwell, sm_11.0) Use 'native' for auto-detection,
# or set explicitly for cross-compilation: -DCMAKE_CUDA_ARCHITECTURES=110
if(CMAKE_CUDA_COMPILER)
  set(CMAKE_CUDA_ARCHITECTURES
      110
      CACHE STRING "CUDA architectures (110 = Blackwell sm_11.0)")
endif()

# ---------------------------------------------------------------------------
# Third-party dependencies, fetched at configure time
#
# Pinned to exact commits and cloned into the build tree by FetchContent, so a
# plain `git clone` of this repo is enough -- no submodules to init or update.
# To build against a local checkout instead (e.g. while hacking on one), pass
# -DFETCHCONTENT_SOURCE_DIR_<NAME>=/path, where <NAME> is CANDLESDK or
# KALMAN_CPP. -DFETCHCONTENT_FULLY_DISCONNECTED=ON reuses what is already in
# build/_deps without touching the network.
#
# Added here, at the top level, rather than in src/: src/CMakeLists.txt sets
# CMAKE_CXX_CLANG_TIDY with --warnings-as-errors=*, which applies to every
# target created after it, and we do not want third-party code linted as ours.
# ---------------------------------------------------------------------------
include(FetchContent)

# CANdle-SDK -- CAN bus motor driver. Only the `candle` library is used.
#
# Of its nested submodules only googletest is fetched: commons/ adds it
# unconditionally. pybind11 and CLI11 belong to the Python bindings and
# candletool, both switched off below.
FetchContent_Declare(
  candlesdk
  GIT_REPOSITORY https://codeberg.org/fatum/CANdle-SDK-CPP17.git
  GIT_TAG 60c4e2c973de5632d81783271d54941a0202b91b
  GIT_SUBMODULES commons/googletest
  EXCLUDE_FROM_ALL)

set(CANDLESDK_BUILD_EXAMPLES
    OFF
    CACHE BOOL "" FORCE)
set(CANDLESDK_BUILD_CANDLETOOL
    OFF
    CACHE BOOL "" FORCE)
set(CANDLE_BUILD_STATIC
    ON
    CACHE BOOL "" FORCE)

# kalman-cpp -- single-target constant-acceleration tracker for the tracking
# loops.
#
# Header-only: the whole API is <kalman/tracker.hpp>, and the only dependency is
# Eigen. As a subproject it already builds no tests, examples or benchmarks and
# exports no install rules; the two switches are pinned anyway so a stale cache
# cannot pull in Google Test or install kalman alongside the turret.
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

# kalman-cpp does its own `find_package(Eigen3 3.4 REQUIRED NO_MODULE)`. Resolve
# it here so a missing Eigen fails with an actionable message instead of one
# surfaced from three directories down inside the dependency.
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

# EXCLUDE_FROM_ALL on both: only what our targets link against gets built.
FetchContent_MakeAvailable(candlesdk kalman_cpp)

add_subdirectory(src)
