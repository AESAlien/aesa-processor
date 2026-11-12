find_package(Threads REQUIRED)

add_library(app_runtime STATIC
    src/channel/beam_channel.cpp
    src/channel/simulator_channel.cpp
    src/channel/console_channel.cpp
    src/channel/target_channel.cpp

    src/config/config.cpp
    src/config/config_assembler.cpp
    src/config/config_loader.cpp
    src/config/config_value_reader.cpp
    src/config/env_file_reader.cpp

    src/log/log_manager.cpp
    src/log/message_log_manager.cpp
    src/log/csv_log_manager.cpp
    src/log/support/timestamped_log_path.cpp

    src/log/logger/beam_event_message_logger.cpp
    src/log/logger/detection_event_csv_logger.cpp

    src/worker/beam_worker.cpp
    src/worker/comm_receive_worker.cpp
    src/worker/simulator_send_worker.cpp
    src/worker/console_send_worker.cpp
    src/worker/target_worker.cpp
    src/worker/worker_error.cpp
)

# 앱 내부 구현과 테스트에만 필요한 경로와 의존성을 지정한다.
function(configure_app_internal target_name)
    target_include_directories(${target_name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(${target_name} PRIVATE
        spdlog::spdlog
        beam
        target
        comm
        Threads::Threads
        magic_enum::magic_enum
    )
endfunction()

configure_app_internal(app_runtime)

add_executable(app
    src/main.cpp
)

configure_app_internal(app)

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

    add_executable(app_communication_test
        test/communication_test.cpp
    )
    configure_app_internal(app_communication_test)

    target_link_libraries(app_communication_test PRIVATE app_runtime GTest::gtest_main)
    gtest_discover_tests(app_communication_test PROPERTIES TIMEOUT 10)

    add_executable(app_config_test
        test/config_test.cpp
    )

    configure_app_internal(app_config_test)

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

    add_executable(app_log_manager_test
        test/beam_event_message_logger_test.cpp
        test/detection_event_csv_test.cpp
        test/log_manager_test.cpp
    )

    target_include_directories(app_log_manager_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/log"
    )

    configure_app_internal(app_log_manager_test)

    target_link_libraries(app_log_manager_test PRIVATE
        app_runtime
        GTest::gtest_main
    )

    gtest_discover_tests(app_log_manager_test)
endif()
