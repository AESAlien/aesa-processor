add_library(math STATIC
    src/geometry/angular_width.cpp
    src/geometry/azimuth.cpp
    src/geometry/elevation.cpp
    src/geometry/cartesian_acceleration.cpp
    src/geometry/cartesian_displacement.cpp
    src/geometry/cartesian_position.cpp
    src/geometry/cartesian_velocity.cpp
    src/quantity/angle.cpp
    src/quantity/length.cpp
    src/geometry/spherical_direction.cpp
    src/linear_algebra/matrix.cpp
    src/linear_algebra/square_matrix.cpp
    src/geometry/spherical_position.cpp
    src/trigonometry.cpp
    src/tolerance.cpp
    src/linear_algebra/vector.cpp
    src/quantity/linear_velocity.cpp
    src/quantity/linear_acceleration.cpp
)

target_include_directories(math PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    add_executable(angle_test test/angle_test.cpp)
    target_link_libraries(angle_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(angle_test)

    add_executable(angular_domain_test test/angular_domain_test.cpp)
    target_link_libraries(angular_domain_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(angular_domain_test)

    add_executable(length_test test/length_test.cpp)
    target_link_libraries(length_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(length_test)

    add_executable(spherical_position_test test/spherical_position_test.cpp)
    target_link_libraries(spherical_position_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(spherical_position_test)

    add_executable(cartesian_displacement_test test/cartesian_displacement_test.cpp)
    target_link_libraries(cartesian_displacement_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(cartesian_displacement_test)

    add_executable(trigonometry_test test/trigonometry_test.cpp)
    target_link_libraries(trigonometry_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(trigonometry_test)

    add_executable(linear_velocity_test test/linear_velocity_test.cpp)
    target_link_libraries(linear_velocity_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(linear_velocity_test)

    add_executable(linear_acceleration_test test/linear_acceleration_test.cpp)
    target_link_libraries(linear_acceleration_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(linear_acceleration_test)

    add_executable(cartesian_acceleration_test test/cartesian_acceleration_test.cpp)
    target_link_libraries(cartesian_acceleration_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(cartesian_acceleration_test)

    add_executable(matrix_test test/matrix_test.cpp)
    target_link_libraries(matrix_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(matrix_test)

    add_executable(square_matrix_test test/square_matrix_test.cpp)
    target_link_libraries(square_matrix_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(square_matrix_test)

    add_executable(vector_test test/vector_test.cpp)
    target_link_libraries(vector_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(vector_test)

    add_executable(value_comparison_test test/value_comparison_test.cpp)
    target_link_libraries(value_comparison_test PRIVATE math GTest::gtest_main)
    gtest_discover_tests(value_comparison_test)
endif()
