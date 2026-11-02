#include <app/log/logger.hpp>

#include <spdlog/async.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <cstdio>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace logging
{

namespace
{

// 초기화와 종료가 동시에 실행되지 않도록 보호한다.
// 일반 로그 기록의 동기화는 spdlog가 담당한다.
std::mutex lifecycleMutex;

std::shared_ptr<spdlog::sinks::basic_file_sink_mt> fileSink;

bool initialized = false;

} // namespace

void initialize(
    const std::string& path,
    std::size_t queueSize)
{
    std::lock_guard<std::mutex> lock(lifecycleMutex);


    if (initialized)
    {
        throw std::logic_error(
            "Logging can only be initialized once");
    }

    if (path.empty() || queueSize == 0)
    {
        throw std::invalid_argument(
            "Invalid logging configuration");
    }

    const auto parent =
        std::filesystem::path(path).parent_path();

    if (!parent.empty())
    {
        std::filesystem::create_directories(parent);
    }

    // 모든 모듈이 공유하는 파일 출력 객체.
    // false는 기존 파일에 이어 쓰는 설정이다.
    auto sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            path,
            false);

    // Parsing
    sink->set_formatter(
        std::make_unique<spdlog::pattern_formatter>(
            "%Y-%m-%dT%H:%M:%S.%eZ "
            "[%n] [%l] [thread=%t] %v",
            spdlog::pattern_time_type::utc));

    try
    {
        // 공용 큐 하나, 로그 기록 스레드 하나.
        spdlog::init_thread_pool(queueSize, 1);

        const auto addLogger =
            [&](const char* name,
                spdlog::level::level_enum level,
                spdlog::async_overflow_policy policy)
        {
            auto logger =
                std::make_shared<spdlog::async_logger>(
                    name,
                    sink,
                    spdlog::thread_pool(),
                    policy);

            logger->set_level(level);

            // INFO 이상은 파일 기록 후 flush한다.
            logger->flush_on(spdlog::level::info);

            // 로깅 자체의 오류는 같은 로거로 재기록하지 않는다.
            logger->set_error_handler(
                [](const std::string& error)
                {
                    std::fprintf(
                        stderr,
                        "[logging] %s\n",
                        error.c_str());
                });

            spdlog::register_logger(logger);
        };

        addLogger(
            "app",
            spdlog::level::info,
            spdlog::async_overflow_policy::discard_new);

        addLogger(
            "beam",
            spdlog::level::info,
            spdlog::async_overflow_policy::discard_new);

        addLogger(
            "target",
            spdlog::level::info,
            spdlog::async_overflow_policy::discard_new);

        addLogger(
            "comm",
            spdlog::level::info,
            spdlog::async_overflow_policy::discard_new);

        addLogger(
            "math",
            spdlog::level::info,
            spdlog::async_overflow_policy::discard_new);

        fileSink = std::move(sink);
        initialized = true;
    }
    catch (...)
    {
        // 초기화 도중 실패하면 생성된 로거와 스레드를 정리한다.
        spdlog::shutdown();
        throw;
    }
}

void shutdown()
{
    std::lock_guard<std::mutex> lock(lifecycleMutex);

    if (!fileSink)
    {
        return;
    }

    // 정상 종료에서는 로그 생산을 먼저 중단한 뒤 호출한다.
    // 등록된 로거와 공용 스레드 풀을 정리한다.
    spdlog::shutdown();

    // DEBUG를 포함하여 파일 버퍼에 남은 내용도 내보낸다.
    fileSink->flush();

    fileSink.reset();
}

} // namespace logging