cmake_minimum_required(VERSION 3.21)
project(
  patronus_tracing
  VERSION 1.0.0
  LANGUAGES CXX)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TRACING_GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0)

add_library(patronus_tracing STATIC src/tracing.cpp)
add_library(patronus::tracing ALIAS patronus_tracing)
target_compile_features(patronus_tracing PUBLIC cxx_std_17)
target_include_directories(patronus_tracing
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(patronus_tracing PUBLIC PkgConfig::TRACING_GSTREAMER
                                              Threads::Threads)

option(PATRONUS_TRACING_BUILD_TESTS "Build standalone tracing test"
       ${PROJECT_IS_TOP_LEVEL})
if(PATRONUS_TRACING_BUILD_TESTS)
  enable_testing()
  add_executable(tracing_smoke tests/smoke.cpp)
  target_link_libraries(tracing_smoke PRIVATE patronus::tracing)
  add_test(NAME tracing_smoke COMMAND tracing_smoke)
endif()
