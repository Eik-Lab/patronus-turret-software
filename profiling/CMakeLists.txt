cmake_minimum_required(VERSION 3.21)
project(
  patronus_profiling
  VERSION 1.0.0
  LANGUAGES CXX)

option(PATRONUS_PROFILING "Enable stdout profiling" OFF)

add_library(patronus_profiling INTERFACE)
add_library(patronus::profiling ALIAS patronus_profiling)
target_include_directories(patronus_profiling
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(patronus_profiling INTERFACE cxx_std_17)

if(PATRONUS_PROFILING)
  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(
      FATAL_ERROR "Profiling resource sampling currently supports Linux only")
  endif()
  find_package(Threads REQUIRED)
  add_library(patronus_profile_recorder STATIC src/profiling.cpp)
  target_include_directories(patronus_profile_recorder
                             PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(patronus_profile_recorder PUBLIC cxx_std_17)
  target_compile_definitions(patronus_profile_recorder
                             PUBLIC PATRONUS_PROFILING=1)
  target_link_libraries(patronus_profile_recorder PUBLIC Threads::Threads
                                                         ${CMAKE_DL_LIBS})
  find_path(NVML_INCLUDE_DIR nvml.h PATHS /usr/local/cuda/include
                                          /usr/local/cuda-13.0/include)
  if(NVML_INCLUDE_DIR)
    target_include_directories(patronus_profile_recorder SYSTEM
                               PRIVATE ${NVML_INCLUDE_DIR})
    target_compile_definitions(patronus_profile_recorder
                               PRIVATE PATRONUS_HAVE_NVML=1)
  else()
    message(
      STATUS "nvml.h not found: GPU profiling metrics will be unavailable")
  endif()
  target_link_libraries(patronus_profiling INTERFACE patronus_profile_recorder)
endif()

option(PATRONUS_PROFILING_BUILD_TESTS "Build standalone profiling smoke test"
       ${PROJECT_IS_TOP_LEVEL})
if(PATRONUS_PROFILING_BUILD_TESTS)
  enable_testing()
  add_executable(profiling_smoke tests/smoke.cpp)
  target_link_libraries(profiling_smoke PRIVATE patronus::profiling)
  add_test(NAME profiling_smoke COMMAND profiling_smoke)
endif()
