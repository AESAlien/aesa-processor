add_library(comm STATIC
    src/controller/comm_controller.cpp
    src/controller/comm_error_handler.cpp
    src/network/message_decoder.cpp
    src/network/message_encoder.cpp
    src/network/message_framer.cpp
    src/network/network_manager.cpp
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
include(GoogleTest)

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
