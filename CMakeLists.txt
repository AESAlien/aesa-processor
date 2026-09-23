cmake_minimum_required(VERSION 3.10)

project(radar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_executable(radar
    src/main.cpp
)

target_include_directories(radar PRIVATE
    include
)

target_link_libraries(radar PRIVATE Threads::Threads)