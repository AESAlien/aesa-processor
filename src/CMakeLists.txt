find_package(Threads REQUIRED)

# 라이브러리 소스는 프로젝트 루트의 third_party/spdlog에 둔다.
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/third_party/spdlog/CMakeLists.txt")
    message(FATAL_ERROR
        "Place spdlog v1.15.3 in third_party/spdlog first"
    )
endif()

function(add_radar_spdlog)
    # 다른 모듈의 BUILD_SHARED_LIBS에는 영향을 주지 않는다.
    set(BUILD_SHARED_LIBS OFF)

    set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_ALL OFF CACHE BOOL "" FORCE)

    set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_EXAMPLE_HO OFF CACHE BOOL "" FORCE)

    set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS_HO OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)

    # 별도 fmt 설치 없이 spdlog에 포함된 fmt를 사용한다.
    set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)
    set(SPDLOG_FMT_EXTERNAL_HO OFF CACHE BOOL "" FORCE)

    # 현재 프로젝트의 C++17을 유지한다.
    set(SPDLOG_USE_STD_FORMAT OFF CACHE BOOL "" FORCE)
    set(SPDLOG_NO_EXCEPTIONS OFF CACHE BOOL "" FORCE)

    add_subdirectory(
        "${PROJECT_SOURCE_DIR}/third_party/spdlog"
        "${CMAKE_BINARY_DIR}/third_party/spdlog"
    )
endfunction()

add_radar_spdlog()

add_library(radar_logging STATIC
    app/src/log/logger.cpp
)

target_include_directories(radar_logging PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/app/include"
)

target_link_libraries(radar_logging PUBLIC
    spdlog::spdlog
    Threads::Threads
)

add_subdirectory(math)
add_subdirectory(beam)
add_subdirectory(target)
add_subdirectory(comm)
add_subdirectory(app)