cmake_minimum_required(VERSION 3.21)
project(
  patronus_logging
  VERSION 1.0.0
  LANGUAGES CXX)

find_package(Threads REQUIRED)
add_library(patronus_logging INTERFACE)
add_library(patronus::logging ALIAS patronus_logging)
target_compile_features(patronus_logging INTERFACE cxx_std_17)
target_include_directories(patronus_logging
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(patronus_logging INTERFACE Threads::Threads)

option(PATRONUS_LOGGING_BUILD_TESTS "Build standalone logging test"
       ${PROJECT_IS_TOP_LEVEL})
if(PATRONUS_LOGGING_BUILD_TESTS)
  enable_testing()
  add_executable(logging_smoke tests/smoke.cpp)
  target_link_libraries(logging_smoke PRIVATE patronus::logging)
  add_test(NAME logging_smoke COMMAND logging_smoke)
endif()
