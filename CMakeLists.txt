cmake_minimum_required(VERSION 3.25)
project(patronus-turret VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
add_subdirectory("${KALMAN_CPP_DIR}" "${CMAKE_BINARY_DIR}/kalman-cpp"
                 EXCLUDE_FROM_ALL)

add_subdirectory(src)
