add_library(target STATIC
    src/controller/target_controller.cpp
    src/calculation/association/detection_matcher.cpp
    src/calculation/tracking/constant_velocity_algorithm.cpp
    src/selector/tentative_selector.cpp
    src/selector/track_selector.cpp
    src/domain/target.cpp
    src/repository/target_repository.cpp
    src/validation/detection_validation.cpp
)

target_link_libraries(target PUBLIC beam math)
target_include_directories(target PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(target_immutable_geometry_test test/immutable_geometry_test.cpp)
target_include_directories(target_immutable_geometry_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(target_immutable_geometry_test PRIVATE target GTest::gtest_main)
gtest_discover_tests(target_immutable_geometry_test)
