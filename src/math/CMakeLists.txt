find_package(GTest REQUIRED)

add_library(math STATIC
    src/angle.cpp
    src/distance.cpp
    src/spherical_direction.cpp
    src/matrix.cpp
    src/square_matrix.cpp
    src/spherical_point.cpp
    src/trigonometry.cpp
    src/vector.cpp
    src/velocity.cpp
)

target_include_directories(math PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

include(GoogleTest)

add_executable(angle_test test/angle_test.cpp)
target_link_libraries(angle_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(angle_test)

add_executable(distance_test test/distance_test.cpp)
target_link_libraries(distance_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(distance_test)

add_executable(spherical_point_test test/spherical_point_test.cpp)
target_link_libraries(spherical_point_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(spherical_point_test)

add_executable(trigonometry_test test/trigonometry_test.cpp)
target_link_libraries(trigonometry_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(trigonometry_test)

add_executable(velocity_test test/velocity_test.cpp)
target_link_libraries(velocity_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(velocity_test)

add_executable(matrix_test test/matrix_test.cpp)
target_link_libraries(matrix_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(matrix_test)

add_executable(square_matrix_test test/square_matrix_test.cpp)
target_link_libraries(square_matrix_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(square_matrix_test)

add_executable(vector_test test/vector_test.cpp)
target_link_libraries(vector_test PRIVATE math GTest::gtest_main)
gtest_discover_tests(vector_test)
