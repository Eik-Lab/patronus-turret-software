cmake_minimum_required(VERSION 3.15)
project(patronus-turret VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# NVIDIA Jetson AGX Thor (Blackwell, sm_11.0)
# Use 'native' for auto-detection, or set explicitly for cross-compilation:
#   -DCMAKE_CUDA_ARCHITECTURES=110
if(CMAKE_CUDA_COMPILER)
  set(CMAKE_CUDA_ARCHITECTURES 110 CACHE STRING "CUDA architectures (110 = Blackwell sm_11.0)")
endif()

add_subdirectory(src)
