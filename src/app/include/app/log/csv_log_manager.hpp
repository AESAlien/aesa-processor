#pragma once

#include <app/log/log_manager.hpp>

#include <cstddef>
#include <filesystem>
#include <set>
#include <string>

namespace app
{
class CsvLogManager final : public LogManager
{
public:
    explicit CsvLogManager(std::size_t queueSize);

    // 헤더와 이후 메시지는 호출자가 CSV 형식으로 구성한다.
    // 헤더와 행에는 로거의 타임스탬프 등 부가 정보를 추가하지 않는다.
    // 기존 파일에 이어 쓰며, 빈 파일에만 헤더를 기록한다.
    void addLogger(
        const std::string& name,
        const std::filesystem::path& filePath,
        const std::string& header
    );

    std::filesystem::path addTimestampedLogger(
        const std::string& name,
        const std::filesystem::path& filePath,
        const std::string& header
    );

private:
    bool validate(
        const std::string& name,
        const std::filesystem::path& path,
        const std::string& header
    ) const;

    void configureLogger(
        spdlog::logger& logger,
        const std::filesystem::path& path,
        const std::string& header
    ) const;
    void onLoggerRegistered(const std::filesystem::path& path);

    std::set<std::filesystem::path> _filePaths;
};
}
