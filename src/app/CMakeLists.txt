find_package(Threads REQUIRED)

add_library(app_runtime STATIC
    src/channel/beam_channel.cpp
    src/channel/comm_channel.cpp
    src/channel/target_channel.cpp

    src/config/config.cpp
    src/config/config_assembler.cpp
    src/config/config_factory.cpp
    src/config/config_value_reader.cpp
    src/config/env_file.cpp

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

target_include_directories(app PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config"
)

set_target_properties(app PROPERTIES
    OUTPUT_NAME radar_worker_test
)

target_link_libraries(app PRIVATE
    app_runtime
)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    add_executable(app_config_test
        test/config_test.cpp
    )
    target_link_libraries(app_config_test PRIVATE
        app_runtime
        GTest::gtest_main
    )
    target_include_directories(app_config_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/config"
    )
    target_compile_definitions(app_config_test PRIVATE
        APP_ENV_EXAMPLE_PATH="${CMAKE_SOURCE_DIR}/.env.example"
    )
    gtest_discover_tests(app_config_test)
endif()
