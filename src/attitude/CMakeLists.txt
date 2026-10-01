find_package(Threads REQUIRED)

add_library(attitude STATIC
    src/transform.cpp
    src/rotation.cpp
    src/updater.cpp
)

target_include_directories(attitude
    PUBLIC  "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_features(attitude PUBLIC cxx_std_17)
target_link_libraries(attitude PUBLIC Threads::Threads)


add_executable(attitude_test test/attitude_test.cpp)
target_link_libraries(attitude_test PRIVATE attitude)
add_test(NAME attitude_test COMMAND attitude_test)