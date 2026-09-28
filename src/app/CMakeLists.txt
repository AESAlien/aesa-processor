add_executable(app src/main.cpp)
set_target_properties(app PROPERTIES OUTPUT_NAME radar)

target_include_directories(app PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(app PRIVATE beam target comm)
