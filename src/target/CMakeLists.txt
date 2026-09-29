add_library(target INTERFACE)
target_link_libraries(target INTERFACE beam)
target_include_directories(target INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
