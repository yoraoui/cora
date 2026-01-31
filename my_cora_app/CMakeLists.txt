cmake_minimum_required(VERSION 3.14)
project(cora_drone LANGUAGES CXX)

# =========================
# C++ standard
# =========================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================
# Build type & optimization
# =========================

# =========================
# Executable
# =========================
add_executable(cora_drone
    solve_rfid.cpp
)

# =========================
# Include directories
# =========================
target_include_directories(cora_drone PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/Optimization/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    /usr/include/eigen3
    /usr/include/suitesparse
)

# =========================
# Library search path
# =========================
target_link_directories(cora_drone PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/lib
)

# =========================
# Link libraries
# =========================


target_link_libraries(cora_drone PRIVATE CORA)


# =========================
# RPATH (runtime linker)
# =========================
set_target_properties(cora_drone PROPERTIES
    BUILD_RPATH "$ORIGIN/build/lib"
)
