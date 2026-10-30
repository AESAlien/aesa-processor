add_library(beam STATIC
    src/controller/beam_controller.cpp

    src/domain/beam_width.cpp

    src/scheduler/attitude_frame.cpp
    src/scheduler/beam_request.cpp
    src/scheduler/beam_request_factory.cpp
    src/scheduler/request_coverage.cpp
    src/scheduler/search_grid.cpp
    src/scheduler/request_queue.cpp
    src/scheduler/scheduler.cpp

    src/attitude/ant_enu_transform.cpp
    src/attitude/radar_attitude.cpp

    src/config/attitude_config.cpp
    src/config/search_grid_config.cpp
    src/config/request_beam_config.cpp
    src/config/request_coverage_config.cpp
    src/config/beam_config.cpp
)

target_include_directories(beam
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(beam PUBLIC math)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    add_executable(beam_config_test
        test/unit/beam_config_test.cpp
        test/unit/attitude_config_test.cpp
        test/unit/request_beam_config_test.cpp
        test/unit/request_coverage_config_test.cpp
        test/unit/search_grid_config_test.cpp
    )
    target_link_libraries(beam_config_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_config_test)

    add_executable(beam_width_test
        test/unit/beam_width_test.cpp
    )
    target_link_libraries(beam_width_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_width_test)

    add_executable(beam_attitude_test
        test/unit/ant_enu_transform_test.cpp
        test/unit/radar_attitude_test.cpp
    )
    target_include_directories(beam_attitude_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_attitude_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_attitude_test)

    add_executable(beam_search_grid_test
        test/unit/search_grid_test.cpp
    )
    target_include_directories(beam_search_grid_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_search_grid_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_search_grid_test)

    add_executable(beam_request_test
        test/unit/beam_request_test.cpp
        test/unit/beam_type_test.cpp
    )
    target_include_directories(beam_request_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_request_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_request_test)

    add_executable(beam_request_factory_test
        test/unit/beam_request_factory_test.cpp
    )
    target_include_directories(beam_request_factory_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_request_factory_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_request_factory_test)

    add_executable(beam_request_queue_test
        test/unit/request_queue_test.cpp
    )
    target_include_directories(beam_request_queue_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_request_queue_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_request_queue_test)

    add_executable(beam_request_coverage_test
        test/unit/request_coverage_test.cpp
    )
    target_include_directories(beam_request_coverage_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_request_coverage_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_request_coverage_test)

    add_executable(beam_scheduler_test
        test/unit/scheduler_test.cpp
    )
    target_include_directories(beam_scheduler_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_scheduler_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_scheduler_test)

    add_executable(beam_controller_test
        test/unit/beam_controller_test.cpp
    )
    target_include_directories(beam_controller_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_controller_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_controller_test)

    add_executable(beam_scheduler_scenario_test
        test/scheduler_scenario_test.cpp
    )
    target_include_directories(beam_scheduler_scenario_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_scheduler_scenario_test PRIVATE
        beam
        GTest::gtest_main
    )
    gtest_discover_tests(beam_scheduler_scenario_test)

    # SearchGrid 값을 출력하는 시각화용 도구
    add_executable(beam_table_dump
        test/dump_beam_table.cpp
    )
    target_include_directories(beam_table_dump PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_table_dump PRIVATE beam)

    # 빔 스케줄을 출력하는 시각화용 도구
    add_executable(beam_schedule_dump
        test/dump_schedule.cpp
    )
    target_include_directories(beam_schedule_dump PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(beam_schedule_dump PRIVATE beam)
endif()
