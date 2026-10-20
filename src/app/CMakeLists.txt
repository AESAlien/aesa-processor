find_package(Threads REQUIRED)

add_library(app_runtime STATIC
    src/channel/beam_channel.cpp
    src/channel/comm_channel.cpp
    src/channel/target_channel.cpp

    src/worker/beam_worker.cpp
    src/worker/comm_worker.cpp
    src/worker/target_worker.cpp
)

target_include_directories(app_runtime PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(app_runtime PUBLIC
    beam
    target
    comm
    Threads::Threads
)

add_executable(app
    src/main.cpp
)

set_target_properties(app PROPERTIES
    OUTPUT_NAME radar_worker_test
)

target_link_libraries(app PRIVATE
    app_runtime
)