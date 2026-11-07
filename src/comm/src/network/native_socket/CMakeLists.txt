add_library(native_socket STATIC native_socket.cpp)
target_include_directories(native_socket PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../.."
)

if(WIN32)
    target_link_libraries(native_socket PUBLIC ws2_32)
endif()
