add_library(comm STATIC
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
target_include_directories(comm_main PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(comm_main PRIVATE comm)

add_executable(comm_sender test/comm_sender.cpp)
target_include_directories(comm_sender PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(comm_sender PRIVATE comm)
