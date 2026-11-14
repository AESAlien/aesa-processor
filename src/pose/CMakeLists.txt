add_library(pose STATIC
    src/domain/ant_enu_transform.cpp
    src/domain/enu_geo_transform.cpp
    src/domain/radar_attitude.cpp
    src/domain/factory/radar_attitude_factory.cpp
    src/config/ant_enu_transform_config.cpp
    src/config/radar_attitude_factory_config.cpp
    src/controller/pose_controller.cpp
)

target_include_directories(pose PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(pose PUBLIC math)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    add_executable(pose_test
        test/ant_enu_transform_test.cpp
        test/enu_geo_transform_test.cpp
        test/pose_config_test.cpp
        test/pose_controller_test.cpp
        test/radar_attitude_test.cpp
    )
    target_link_libraries(pose_test PRIVATE pose GTest::gtest_main)
    gtest_discover_tests(pose_test)
endif()
