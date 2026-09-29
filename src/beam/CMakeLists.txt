add_library(beam STATIC 
    src/timer/timer.cpp
    src/scheduler/scheduler.cpp
)

target_include_directories(beam PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(beam PUBLIC attitude)
