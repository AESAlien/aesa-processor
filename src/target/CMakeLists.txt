add_library(target STATIC
    src/config/target_config.cpp
    src/controller/target_controller.cpp
    src/calculation/association/detection_matcher.cpp
    src/calculation/tracking/constant_velocity_algorithm.cpp
    src/calculation/tracking/constant_acceleration_algorithm.cpp
    src/selector/tentative_selector.cpp
    src/selector/track_selector.cpp
    src/domain/target.cpp
    src/domain/tracking_state.cpp
    src/repository/target_repository.cpp
)

target_link_libraries(target PUBLIC beam math)
target_include_directories(target PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    add_executable(target_immutable_geometry_test test/immutable_geometry_test.cpp)
    target_include_directories(target_immutable_geometry_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_immutable_geometry_test PRIVATE target GTest::gtest_main)
    gtest_discover_tests(target_immutable_geometry_test)

    add_executable(target_domain_validation_test test/domain_validation_test.cpp)
    target_include_directories(target_domain_validation_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_domain_validation_test PRIVATE target GTest::gtest_main)
    gtest_discover_tests(target_domain_validation_test)

    add_executable(target_tracking_association_test test/tracking_association_test.cpp)
    target_include_directories(target_tracking_association_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_tracking_association_test PRIVATE target GTest::gtest_main)
    gtest_discover_tests(target_tracking_association_test)

    add_executable(target_tracking_state_test test/tracking_state_test.cpp)
    target_include_directories(target_tracking_state_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_tracking_state_test PRIVATE target GTest::gtest_main)
    gtest_discover_tests(target_tracking_state_test)

    add_executable(target_constant_acceleration_algorithm_test test/constant_acceleration_algorithm_test.cpp)
    target_include_directories(target_constant_acceleration_algorithm_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_constant_acceleration_algorithm_test PRIVATE target GTest::gtest_main)

    gtest_discover_tests(target_constant_acceleration_algorithm_test)

    add_executable(target_kalman_controller_test test/kalman_controller_test.cpp)
    target_include_directories(target_kalman_controller_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_kalman_controller_test PRIVATE target GTest::gtest_main)

    gtest_discover_tests(target_kalman_controller_test)

    add_executable(target_track_reporting_limit_test test/track_reporting_limit_test.cpp)
    target_include_directories(target_track_reporting_limit_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_track_reporting_limit_test PRIVATE target GTest::gtest_main)

    gtest_discover_tests(target_track_reporting_limit_test)
endif()
