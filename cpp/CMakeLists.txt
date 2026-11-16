cmake_minimum_required(VERSION 3.16)
project(pmsm_foc_q15 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(FOC_WERROR "Treat warnings as errors" OFF)
option(FOC_SANITIZE "Build with address + undefined-behaviour sanitizers" OFF)

add_library(foc_q15 STATIC src/current_controller_q15.cpp)
target_include_directories(foc_q15 PUBLIC include)
target_compile_options(foc_q15 PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wsign-conversion)
if(FOC_WERROR)
  target_compile_options(foc_q15 PRIVATE -Werror)
endif()
if(FOC_SANITIZE)
  target_compile_options(foc_q15 PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(foc_q15 PUBLIC -fsanitize=address,undefined)
endif()

enable_testing()
add_executable(test_q15 tests/test_q15.cpp)
target_link_libraries(test_q15 PRIVATE foc_q15)
add_test(NAME test_q15 COMMAND test_q15)

# Stage 6 tools: replay recorded vectors and time one control iteration.
add_executable(replay_q15 tools/replay_q15.cpp)
target_link_libraries(replay_q15 PRIVATE foc_q15)
add_executable(bench_q15 tools/bench_q15.cpp)
target_link_libraries(bench_q15 PRIVATE foc_q15)
