add_library(target STATIC
    src/controller/target_controller.cpp
    src/calculation/association/detection_matcher.cpp
    src/calculation/tracking/constant_velocity_algorithm.cpp
    src/selector/tentative_selector.cpp
    src/selector/track_selector.cpp
    src/domain/target.cpp
    src/repository/target_repository.cpp
)

target_link_libraries(target PUBLIC beam)
target_include_directories(target PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
