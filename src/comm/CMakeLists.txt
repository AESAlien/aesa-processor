add_subdirectory(src/socket)

add_library(comm STATIC
    src/config/comm_config.cpp
    src/config/comm_endpoint.cpp
    src/controller/comm_controller.cpp
    src/controller/comm_error_handler.cpp
    src/message/message_decoder.cpp
    src/message/message_encoder.cpp
    src/message/message_framer.cpp
    src/network/network_manager.cpp
    src/network/peer_connection.cpp
    src/network/session.cpp
    src/socket/socket_error.cpp
    src/socket/socket_selector.cpp
    src/socket/socket.cpp
    src/socket/listen_socket.cpp
    src/socket/connected_socket.cpp
    src/network/tcp_server.cpp
)
target_include_directories(comm
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(comm PUBLIC
    beam
    target
    spdlog::spdlog
)
target_link_libraries(comm PRIVATE native_socket Threads::Threads)

if(BUILD_TESTING)
    add_executable(comm_main test/comm_main.cpp)
    target_link_libraries(comm_main PRIVATE comm)

    add_executable(comm_sender test/comm_sender.cpp)
    target_include_directories(comm_sender PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_sender PRIVATE comm)

    find_package(GTest REQUIRED)
    find_package(Threads REQUIRED)
    include(GoogleTest)

    add_library(comm_native_socket_test_support STATIC
        test/native_socket/native_socket_test_support.cpp
    )
    target_include_directories(comm_native_socket_test_support PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/test"
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_native_socket_test_support PRIVATE
        native_socket
        Threads::Threads
    )

    add_executable(comm_message_decoder_test
        test/unit/message_decoder_test.cpp
    )
    target_include_directories(comm_message_decoder_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_message_decoder_test PRIVATE
        comm
        GTest::gtest_main
    )
    gtest_discover_tests(comm_message_decoder_test)

    add_executable(comm_socket_test test/socket_test.cpp)
    target_include_directories(comm_socket_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_socket_test PRIVATE
        comm
        GTest::gtest_main
        Threads::Threads
        comm_native_socket_test_support
    )
    gtest_discover_tests(comm_socket_test)

    add_executable(comm_socket_error_test test/socket_error_test.cpp)
    target_include_directories(comm_socket_error_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_socket_error_test PRIVATE
        comm
        GTest::gtest_main
        comm_native_socket_test_support
    )
    gtest_discover_tests(comm_socket_error_test)

    add_executable(comm_config_test test/unit/comm_config_test.cpp)
    target_link_libraries(comm_config_test PRIVATE
        comm
        GTest::gtest_main
    )
    gtest_discover_tests(comm_config_test)

    add_executable(comm_message_encoder_test test/unit/message_encoder_test.cpp)
    target_include_directories(comm_message_encoder_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_message_encoder_test PRIVATE
        comm
        GTest::gtest_main
    )
    gtest_discover_tests(comm_message_encoder_test)

    add_executable(comm_message_framer_test test/unit/message_framer_test.cpp)
    target_include_directories(comm_message_framer_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_message_framer_test PRIVATE
        comm
        GTest::gtest_main
    )
    gtest_discover_tests(comm_message_framer_test)
endif()
