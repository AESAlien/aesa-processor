add_library(target STATIC src/hello.cpp)
target_include_directories(target PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
