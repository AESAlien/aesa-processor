add_library(target STATIC
    src/controller/target_controller.cpp
    src/candidate/candidate_selector.cpp
    src/repository/track_repository.cpp
)

target_link_libraries(target PUBLIC beam)
target_include_directories(target PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(target_search_test test/target_controller_test.cpp)
target_include_directories(target_search_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(target_search_test PRIVATE target GTest::gtest_main)
gtest_discover_tests(target_search_test)

add_executable(target_search_demo test/search_demo.cpp)
target_include_directories(target_search_demo PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(target_search_demo PRIVATE target)
