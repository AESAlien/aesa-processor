add_library(target STATIC
    src/config/association_config.cpp
    src/config/measurement_config.cpp
    src/config/tracking_config.cpp
    src/config/track_management_config.cpp
    src/config/target_config.cpp
    src/controller/target_controller.cpp
    src/service/detection_handler.cpp
    src/service/detection_matcher.cpp
    src/algorithm/nds_algorithm.cpp
    src/algorithm/constant_velocity_algorithm.cpp
    src/algorithm/constant_acceleration_algorithm.cpp
    src/service/tentative_selector.cpp
    src/service/track_selector.cpp
    src/domain/detection_result.cpp
    src/domain/observation.cpp
    src/domain/tentative_track.cpp
    src/domain/confirmed_track.cpp
    src/domain/factory/observation_factory.cpp
    src/domain/factory/detection_result_factory.cpp
    src/domain/factory/track_factory.cpp
    src/dto/factory/track_event_factory.cpp
    src/domain/track_state.cpp
    src/repository/track_repository.cpp
)

target_link_libraries(target PUBLIC beam math)
target_include_directories(target PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    add_executable(target_track_repository_test test/track_repository_test.cpp)
    target_include_directories(target_track_repository_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_track_repository_test PRIVATE target GTest::gtest_main)
    gtest_discover_tests(target_track_repository_test)

    add_executable(target_detection_handler_test test/detection_handler_test.cpp)
    target_include_directories(target_detection_handler_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_detection_handler_test PRIVATE target GTest::gtest_main)
    gtest_discover_tests(target_detection_handler_test)

    add_executable(target_nds_algorithm_test test/nds_algorithm_test.cpp)
    target_include_directories(target_nds_algorithm_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_nds_algorithm_test PRIVATE target GTest::gtest_main)
    gtest_discover_tests(target_nds_algorithm_test)

    add_executable(target_config_test test/target_config_test.cpp)
    target_include_directories(target_config_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_config_test PRIVATE target GTest::gtest_main)
    gtest_discover_tests(target_config_test)

    add_executable(target_lifecycle_test test/target_lifecycle_test.cpp)
    target_include_directories(target_lifecycle_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(target_lifecycle_test PRIVATE target GTest::gtest_main)
    gtest_discover_tests(target_lifecycle_test)

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
    target_link_libraries(target PUBLIC radar_logging)

    gtest_discover_tests(target_track_reporting_limit_test)
endif()
