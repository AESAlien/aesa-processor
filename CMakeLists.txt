cmake_minimum_required(VERSION 3.10)

project(radar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/utf-8)
endif()

include(CTest)
add_subdirectory(src)
