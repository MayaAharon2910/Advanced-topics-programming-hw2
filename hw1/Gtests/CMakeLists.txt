add_executable(drone_mapper_tests
    test_main.cpp
)

target_link_libraries(drone_mapper_tests
    PRIVATE
        hw1_lib
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(drone_mapper_tests)
