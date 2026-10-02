add_library(beam STATIC 
    src/timer/timer.cpp
    src/scheduler/scheduler.cpp
    src/attitude/rotation.cpp
    src/attitude/transform.cpp
    src/attitude/updater.cpp
)

target_include_directories(beam PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(beam PUBLIC math)

add_executable(attitude_test test/attitude_test.cpp)
target_link_libraries(attitude_test PRIVATE beam)
add_test(NAME attitude_test COMMAND attitude_test)
