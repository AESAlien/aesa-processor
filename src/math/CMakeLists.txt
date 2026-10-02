find_package(GTest REQUIRED)

add_library(math STATIC
    src/angle.cpp
    src/matrix.cpp
)

target_include_directories(math PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

include(GoogleTest)

add_executable(angle_test test/angle_test.cpp)
target_link_libraries(angle_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(angle_test)

add_executable(matrix_test test/matrix_test.cpp)
target_link_libraries(matrix_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(matrix_test)