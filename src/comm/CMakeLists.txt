add_library(comm STATIC src/hello.cpp)
target_include_directories(comm PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(comm PUBLIC
    beam
    target
)
