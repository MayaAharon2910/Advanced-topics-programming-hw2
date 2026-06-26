cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# Enforce strict compilation flags for grader (-Werror etc.)
add_compile_options(-Wall -Wextra -Werror -pedantic)


find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED) # new!

add_library(drone_mapper
    src/DroneControlImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/Logger.cpp
    src/YamlConfig.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

# Compile the yaml example file
add_executable(example_yml cpp_yaml_example/main.cpp)
target_link_libraries(example_yml PRIVATE yaml-cpp::yaml-cpp)

# Tests
enable_testing()
find_package(GTest CONFIG REQUIRED)

add_executable(drone_mapper_simulation_test
    tests/components/MockLidar_test.cpp
    tests/components/SimulationRun_test.cpp
    tests/components/DroneControl_test.cpp
    tests/components/MapsComparison_test.cpp
    tests/components/MissionControl_test.cpp
    tests/components/MappingAlgorithm_test.cpp
    tests/components/SimulationManager_test.cpp
    tests/integration/full_flow_real_mapping.cpp
    tests/integration/full_flow_mock_mapping.cpp
    tests/integration/integration_tests.cpp
    tests/integration/error_handling_tests.cpp
    tests/integration/cartesian_product_test.cpp
    tests/integration/hw1_edge_cases_test.cpp
    tests/integration/benchmark_map_test.cpp
)
target_link_libraries(drone_mapper_simulation_test PRIVATE drone_mapper GTest::gtest_main GTest::gmock)
add_test(NAME drone_mapper_simulation_test COMMAND drone_mapper_simulation_test)
