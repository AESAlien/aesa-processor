add_library(attitude STATIC src/attitude.cpp)

target_include_directories(attitude PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
