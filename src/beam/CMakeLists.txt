add_library(beam STATIC src/hello.cpp)
target_include_directories(beam PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
