#pragma once

#include <chrono>
#include <filesystem>

namespace app
{
// 빈 파일을 배타적으로 생성해 실행별 로그 경로를 확보한다.
std::filesystem::path reserveTimestampedLogPath(
    const std::filesystem::path& basePath,
    std::chrono::system_clock::time_point started = std::chrono::system_clock::now()
);
}
