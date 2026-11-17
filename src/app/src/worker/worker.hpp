#pragma once

#include <spdlog/spdlog.h>

#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace app
{
template <typename Input, typename Output>
class Worker
{
public:
    virtual ~Worker() = default;

    void run()
    {
        _logger->info("Worker started");

        while (true)
        {
            try
            {
                runOnce();
                continue;
            }
            catch (const std::exception& error)
            {
                _logger->error("{}", error.what());
            }
            catch (...)
            {
                _logger->error("Unknown worker failure");
            }

            std::this_thread::sleep_for(_retryDelay);
        }
    }

    // 입력 하나를 읽어 처리하고, 결과가 있으면 출력한다. 입력이 없으면 기다린다.
    void runOnce()
    {
        Input input = readInput();
        if (auto output = process(std::move(input)))
        {
            writeOutput(std::move(*output));
        }
    }

protected:
    Worker(
        std::shared_ptr<spdlog::logger> logger,
        std::chrono::milliseconds retryDelay = std::chrono::milliseconds{0}
    ) : _logger(std::move(logger)),
        _retryDelay(retryDelay)
    {
        if (!_logger)
        {
            throw std::invalid_argument("Worker logger cannot be null");
        }
    }

    virtual Input readInput() = 0;
    virtual std::optional<Output> process(Input input) = 0;
    virtual void writeOutput(Output output) {}

private:
    std::shared_ptr<spdlog::logger> _logger;
    std::chrono::milliseconds _retryDelay;
};
} // namespace app
