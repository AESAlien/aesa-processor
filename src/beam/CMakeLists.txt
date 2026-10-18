add_library(beam STATIC
    src/controller/beam_controller.cpp
    src/scheduler/beam_request.cpp
    src/scheduler/search_beam_table.cpp
    src/scheduler/request_queue.cpp
    src/scheduler/scheduler.cpp
    src/attitude/ant_enu_transform.cpp
    src/attitude/radar_attitude.cpp
    src/config/attitude_config.cpp
    src/config/search_grid_config.cpp
    src/config/request_beam_config.cpp
    src/config/beam_config.cpp
)

target_include_directories(beam
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(beam PUBLIC math)

find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(beam_config_test test/config_test.cpp)
target_link_libraries(beam_config_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_config_test)

add_executable(beam_attitude_test test/attitude_test.cpp)
target_include_directories(beam_attitude_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(beam_attitude_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_attitude_test)

# SearchBeamTable은 beam 내부(private) 헤더이므로 테스트에서만 src/scheduler를 추가로 include
add_executable(beam_search_beam_table_test test/search_beam_table_test.cpp)
target_include_directories(beam_search_beam_table_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/scheduler"
)
target_link_libraries(beam_search_beam_table_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_search_beam_table_test)

add_executable(beam_request_test test/beam_request_test.cpp)
target_include_directories(beam_request_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(beam_request_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_request_test)

add_executable(beam_request_queue_test test/request_queue_test.cpp)
target_include_directories(beam_request_queue_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(beam_request_queue_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_request_queue_test)

add_executable(beam_scheduler_test test/scheduler_test.cpp)
target_include_directories(beam_scheduler_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(beam_scheduler_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_scheduler_test)

add_executable(beam_controller_test test/beam_controller_test.cpp)
target_include_directories(beam_controller_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(beam_controller_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_controller_test)

# 시각화 도구(beam_schedule_dump)가 보여 주는 시나리오를 고정하는 테스트. schedule_scenario.hpp 를 도구와 공유한다.
add_executable(beam_scheduler_scenario_test test/scheduler_scenario_test.cpp)
target_include_directories(beam_scheduler_scenario_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(beam_scheduler_scenario_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_scheduler_scenario_test)

# 시각화용 도구: test/visualize_beam_table.py 가 실행해서 SearchBeamTable 값을 읽는다
add_executable(beam_table_dump test/dump_beam_table.cpp)
target_include_directories(beam_table_dump PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/scheduler"
)
target_link_libraries(beam_table_dump PRIVATE beam)

# 시각화용 도구: test/visualize_schedule.py 가 실행해서 Scheduler 동작 결과를 읽는다
add_executable(beam_schedule_dump test/dump_schedule.cpp)
target_include_directories(beam_schedule_dump PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(beam_schedule_dump PRIVATE beam)
