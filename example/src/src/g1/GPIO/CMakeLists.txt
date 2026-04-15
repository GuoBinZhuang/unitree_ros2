cmake_minimum_required(VERSION 3.16)
project(estop_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_LIBGPIOD "Build with libgpiod backend (must be ON in this source tree)" ON)

if(NOT USE_LIBGPIOD)
	message(FATAL_ERROR "USE_LIBGPIOD=OFF is not supported by current source tree; configure with -DUSE_LIBGPIOD=ON")
endif()

# ─────────────────────────────────────────────────────────
# 仅使用 libgpiod 后端
#   sudo apt install gpiod libgpiod-dev
#   cmake -S . -B build
#   cmake --build build
#   ./build/estop_demo
# ─────────────────────────────────────────────────────────

add_executable(estop_demo main.cpp)
target_include_directories(estop_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(estop_demo PRIVATE USE_LIBGPIOD=$<BOOL:${USE_LIBGPIOD}>)

find_package(Threads REQUIRED)
target_link_libraries(estop_demo PRIVATE Threads::Threads)

find_library(GPIOD_LIB gpiod REQUIRED)
target_link_libraries(estop_demo PRIVATE ${GPIOD_LIB})
message(STATUS "Backend: libgpiod (${GPIOD_LIB})")

target_compile_options(estop_demo PRIVATE -O2 -Wall -Wextra)
