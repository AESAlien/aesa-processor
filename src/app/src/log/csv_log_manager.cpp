#include "log/csv_log_manager.hpp"

#include "support/timestamped_log_path.hpp"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <cstdio>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <system_error>

namespace app
{
namespace
{
void errorHandler(const std::string& error)
{
    std::fprintf(stderr, "[CsvLogManager] %s\n", error.c_str());
}
}

CsvLogManager::CsvLogManager(std::size_t queueSize)
  : LogManager(queueSize)
{
}

void CsvLogManager::addLogger(
    const std::string& name,
    const std::filesystem::path& filePath,
    const std::string& header
) {
    const auto path = std::filesystem::weakly_canonical(filePath);
    LogManager::addLogger(
        name,
        spdlog::level::info,
        spdlog::async_overflow_policy::block,
        std::bind(
            &CsvLogManager::configureLogger,
            this,
            std::placeholders::_1,
            std::cref(path),
            std::cref(header)
        ),
        std::bind(&CsvLogManager::onLoggerRegistered, this, std::cref(path))
    );
}

std::filesystem::path CsvLogManager::addTimestampedLogger(
    const std::string& name,
    const std::filesystem::path& filePath,
    const std::string& header
) {
    const auto path = reserveTimestampedLogPath(filePath);
    try
    {
        addLogger(name, path, header);
    }
    catch (...)
    {
        std::error_code error;
        std::filesystem::remove(path, error);
        throw;
    }
    return path;
}

bool CsvLogManager::validate(
    const std::string& name,
    const std::filesystem::path& path,
    const std::string& header
) const {
    if (path.empty() || header.empty() || header.find_first_of("\r\n") != std::string::npos)
    {
        throw std::invalid_argument("Invalid CSV logging configuration");
    }
    if (spdlog::get(name))
    {
        throw std::invalid_argument("Logger name is already registered");
    }
    const bool exists = std::filesystem::exists(path);
    for (const auto& filePath : _filePaths)
    {
        if (filePath == path ||
            (exists && std::filesystem::equivalent(filePath, path)))
        {
            throw std::invalid_argument("CSV file already has a logger");
        }
    }
    if (!exists || std::filesystem::file_size(path) == 0)
    {
        return true;
    }

    std::ifstream file(path);

    std::string existingHeader;
    if (!std::getline(file, existingHeader))
    {
        throw std::runtime_error("Cannot read CSV header");
    }
    if (!existingHeader.empty() && existingHeader.back() == '\r')
    {
        existingHeader.pop_back();
    }
    if (existingHeader != header)
    {
        throw std::invalid_argument("CSV header does not match existing file");
    }

    file.clear();
    file.seekg(-1, std::ios::end);
    if (file.get() != '\n')
    {
        throw std::invalid_argument("CSV file must end with a newline before appending");
    }
    return false;
}

void CsvLogManager::configureLogger(
    spdlog::logger& logger,
    const std::filesystem::path& path,
    const std::string& header
) const {
    const bool needsHeader = validate(logger.name(), path, header);

    auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path.string(), false);
    sink->set_pattern("%v");

    if (needsHeader)
    {
        // 초기 헤더 쓰기 실패는 등록 실패로 전달한다.
        sink->log(spdlog::details::log_msg(logger.name(), spdlog::level::info, header));
        sink->flush();
    }
    
    logger.sinks().push_back(sink);
    logger.flush_on(spdlog::level::info);
    logger.set_error_handler(errorHandler);
}

void CsvLogManager::onLoggerRegistered(const std::filesystem::path& path)
{
    _filePaths.insert(path);
}
}
