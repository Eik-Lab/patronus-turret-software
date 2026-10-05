cmake_minimum_required(VERSION 3.25)
project(patronus-turret VERSION 1.0.0 LANGUAGES C CXX)

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

# NVIDIA Jetson AGX Thor (Blackwell, sm_11.0)
# Use 'native' for auto-detection, or set explicitly for cross-compilation:
#   -DCMAKE_CUDA_ARCHITECTURES=110
if(CMAKE_CUDA_COMPILER)
  set(CMAKE_CUDA_ARCHITECTURES 110 CACHE STRING "CUDA architectures (110 = Blackwell sm_11.0)")
endif()

# ---------------------------------------------------------------------------
#  kalman-cpp -- IMM target filter for the tracking loops
#
#  Added here, at the top level, rather than in src/: src/CMakeLists.txt sets
#  CMAKE_CXX_CLANG_TIDY with --warnings-as-errors=* before it pulls in
#  deps/CANdle-SDK, and we do not want that third-party code linted as ours.
#
#  The header-only core (kalman/core/imm.hpp, kalman/models/*) is what the
#  tracker uses. Deliberate deviations from the upstream defaults:
#
#    KALMAN_USE_TRACY=OFF     Tracy is attached to the target as a PUBLIC
#                             dependency and opens a localhost capture socket.
#                             Not wanted in a deployed control system.
#    KALMAN_BUILD_APPS=OFF    Otherwise OpenCV 4 (with dnn/highgui) becomes a
#                             hard requirement of the turret build.
#    KALMAN_BUILD_TESTS=OFF   Otherwise Google Test becomes a hard requirement.
#    KALMAN_BUILD_BENCHMARKS=OFF  Otherwise Google Benchmark is required.
#
#  KALMAN_CPP_DIR lets the source be relocated without editing this file.
# ---------------------------------------------------------------------------
set(KALMAN_CPP_DIR "${PROJECT_SOURCE_DIR}/deps/kalman-cpp"
    CACHE PATH "Path to the kalman-cpp source tree")

if(NOT EXISTS "${KALMAN_CPP_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "kalman-cpp not found at '${KALMAN_CPP_DIR}'.\n"
    "Expected a submodule at deps/kalman-cpp, or pass "
    "-DKALMAN_CPP_DIR=/path/to/kalman-cpp.")
endif()

set(KALMAN_USE_TRACY OFF CACHE BOOL "" FORCE)
set(KALMAN_BUILD_APPS OFF CACHE BOOL "" FORCE)
set(KALMAN_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(KALMAN_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)

# kalman-cpp does its own `find_package(Eigen3 3.4 REQUIRED NO_MODULE)`. Resolve
# it here so a missing Eigen fails with an actionable message instead of one
# surfaced from three directories down inside the dependency.
if(NOT TARGET Eigen3::Eigen AND NOT Eigen3_FOUND)
  find_package(Eigen3 3.4 QUIET NO_MODULE)
endif()
if(NOT Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR
    "Eigen3 >= 3.4 not found — required by deps/kalman-cpp.\n"
    "  nix develop                       # the flake devShell provides eigen\n"
    "  sudo apt-get install libeigen3-dev\n"
    "  -DEigen3_DIR=/path/to/eigen/share/eigen3/cmake")
endif()

add_subdirectory("${KALMAN_CPP_DIR}" "${CMAKE_BINARY_DIR}/kalman-cpp"
                 EXCLUDE_FROM_ALL)

add_subdirectory(src)
