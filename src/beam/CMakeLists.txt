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
