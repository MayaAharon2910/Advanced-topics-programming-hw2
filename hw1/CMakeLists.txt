cmake_minimum_required(VERSION 3.20)
project(hw1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(mp-units 2.5.0 CONFIG REQUIRED)

add_library(hw1_lib
    src/Map3D.cpp
    src/MockLidarSensor.cpp
    src/MockPositionSensor.cpp
    src/MockMovementDriver.cpp
    src/FileUtils.cpp
    src/Drone.cpp
    src/MapExport.cpp
    src/Logger.cpp
)

target_compile_features(hw1_lib PUBLIC cxx_std_20)

target_include_directories(hw1_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(hw1_lib
    PUBLIC
        mp-units::mp-units
)

# Course submission flags: keep warnings strict and portable.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(hw1_lib PRIVATE -Wall -Wextra -Werror -pedantic)
endif()

add_executable(drone_mapper src/main.cpp)
target_link_libraries(drone_mapper PRIVATE hw1_lib)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(drone_mapper PRIVATE -Wall -Wextra -Werror -pedantic)
endif()

# ── Unit tests ────────────────────────────────────────────────────────────────
option(BUILD_TESTING "Build GTest unit-test suite" OFF)

if(BUILD_TESTING)
    find_package(GTest CONFIG REQUIRED)
    
    enable_testing()
    add_subdirectory(Gtests)
endif()
