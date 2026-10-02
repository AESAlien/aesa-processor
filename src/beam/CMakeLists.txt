add_library(beam STATIC 
    src/scheduler/beam_table.cpp
    src/scheduler/scheduler.cpp
    src/attitude/rotation.cpp
    src/attitude/transform.cpp
    src/attitude/attitude_transform.cpp
)

target_include_directories(beam PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(beam PUBLIC math)

find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(beam_attitude_test test/attitude_test.cpp)
target_link_libraries(beam_attitude_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_attitude_test)

# BeamTable은 beam 내부(private) 헤더이므로 테스트에서만 src/scheduler를 추가로 include
add_executable(beam_table_test test/beam_table_test.cpp)
target_include_directories(beam_table_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/scheduler")
target_link_libraries(beam_table_test PRIVATE beam GTest::gtest_main)
gtest_discover_tests(beam_table_test)

# 시각화용 도구: test/visualize_beam_table.py 가 실행해서 BeamTable 값을 읽는다
add_executable(beam_table_dump test/dump_beam_table.cpp)
target_include_directories(beam_table_dump PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/scheduler")
target_link_libraries(beam_table_dump PRIVATE beam)
