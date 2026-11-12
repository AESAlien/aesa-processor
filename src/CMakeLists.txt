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

# magic_enum v0.9.7: https://github.com/Neargye/magic_enum/releases/tag/v0.9.7
add_library(magic_enum INTERFACE)
add_library(magic_enum::magic_enum ALIAS magic_enum)
target_include_directories(magic_enum SYSTEM INTERFACE
    "${PROJECT_SOURCE_DIR}/third_party/magic_enum/include"
)

add_subdirectory(math)
add_subdirectory(beam)
add_subdirectory(target)
add_subdirectory(comm)
add_subdirectory(app)
