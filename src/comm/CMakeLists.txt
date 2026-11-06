add_library(comm STATIC
    src/config/comm_config.cpp
    src/config/comm_endpoint.cpp
    src/controller/comm_controller.cpp
    src/controller/comm_error_handler.cpp
    src/network/message_decoder/message_decoder.cpp
    src/network/message_encoder/message_encoder.cpp
    src/network/message_framer/message_framer.cpp
    src/network/network_manager/network_manager.cpp
    src/network/peer_connection/peer_connection.cpp
    src/network/session/session.cpp
    src/network/socket_error/socket_error.cpp
    src/network/socket_selector/socket_selector.cpp
    src/network/tcp_socket/tcp_socket.cpp
    src/network/tcp_server/tcp_server.cpp
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
)

if(WIN32)
    target_link_libraries(comm PUBLIC ws2_32)
endif()

if(BUILD_TESTING)
    add_executable(comm_main test/comm_main.cpp)
    target_link_libraries(comm_main PRIVATE comm)

    add_executable(comm_sender test/comm_sender.cpp)
    target_include_directories(comm_sender PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_sender PRIVATE comm)

    add_executable(comm_message_decoder_test test/message_decoder_test.cpp)
    target_include_directories(comm_message_decoder_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_message_decoder_test PRIVATE
        comm
    )

    find_package(GTest REQUIRED)
    find_package(Threads REQUIRED)
    include(GoogleTest)

    add_executable(comm_tcp_socket_test test/tcp_socket_test.cpp)
    target_include_directories(comm_tcp_socket_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_tcp_socket_test PRIVATE
        comm
        GTest::gtest_main
        Threads::Threads
    )
    gtest_discover_tests(comm_tcp_socket_test)

    add_executable(comm_socket_error_test test/socket_error_test.cpp)
    target_include_directories(comm_socket_error_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_socket_error_test PRIVATE
        comm
        GTest::gtest_main
    )
    gtest_discover_tests(comm_socket_error_test)

    add_executable(comm_config_test test/comm_config_test.cpp)
    target_link_libraries(comm_config_test PRIVATE
        comm
        GTest::gtest_main
    )
    gtest_discover_tests(comm_config_test)

    add_executable(comm_error_handler_test test/comm_error_handler_test.cpp)
    target_include_directories(comm_error_handler_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_error_handler_test PRIVATE
        comm
        GTest::gtest_main
    )
    gtest_discover_tests(comm_error_handler_test)

    add_executable(comm_message_encoder_test test/message_encoder_test.cpp)
    target_include_directories(comm_message_encoder_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_message_encoder_test PRIVATE
        comm
        GTest::gtest_main
    )
    gtest_discover_tests(comm_message_encoder_test)

    add_executable(comm_message_framer_test test/message_framer_test.cpp)
    target_include_directories(comm_message_framer_test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(comm_message_framer_test PRIVATE
        comm
        GTest::gtest_main
    )
    gtest_discover_tests(comm_message_framer_test)
endif()
