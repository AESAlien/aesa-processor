#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <app/channel/target_channel.hpp>

#include <app/log/logger.hpp>

#include <app/worker/beam_worker.hpp>
#include <app/worker/comm_worker.hpp>
#include <app/worker/target_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <beam/controller/beam_controller.hpp>
#include <comm/controller/comm_controller.hpp>
#include <target/controller/target_controller.hpp>

#include <spdlog/spdlog.h>

#include "config_loader.hpp"

#include <array>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{

constexpr std::chrono::milliseconds BEAM_PERIOD{10};

volatile std::sig_atomic_t stopRequested = 0;

void requestStop(int) noexcept
{
    // 시그널 처리 함수에서는 플래그만 변경한다.
    stopRequested = 1;
}

void printEndpoint(
    const char* label,
    const comm::CommEndpoint& endpoint)
{
    std::cout
        << label
        << endpoint.address()
        << ':'
        << endpoint.port()
        << '\n';
}

[[noreturn]] void exitAfterLogging(int status) noexcept
{
    try
    {
        logging::shutdown();
    }
    catch (const std::exception& error)
    {
        app::reportWorkerError(
            "main",
            error.what());

        status = EXIT_FAILURE;
    }
    catch (...)
    {
        app::reportWorkerError(
            "main",
            "Logging shutdown failed");

        status = EXIT_FAILURE;
    }

    // 현재 통신 컨트롤러에는 수신 대기 취소 API가 없다.
    // 로그를 정리한 뒤 프로세스를 종료한다.
    std::_Exit(status);
}

template <typename Action>
void runWorker(
    const char* name,
    Action action)
{
    try
    {
        if (auto logger = spdlog::get(app::workerModule(name)))
        {
            logger->info(
                "[worker={}] Worker started",
                name);
        }

        action();

        app::reportWorkerError(
            name,
            "Worker returned unexpectedly");
    }
    catch (const std::exception& error)
    {
        app::reportWorkerError(
            name,
            error.what());
    }
    catch (...)
    {
        app::reportWorkerError(
            name,
            "Unknown worker failure");
    }

    // 일반적인 모듈 예외는 각 워커 내부에서 처리한다.
    // 여기까지 도달하면 워커 실행 자체가 끝난 상황이다.
    exitAfterLogging(EXIT_FAILURE);
}

} // namespace

int main()
{
    // 스레드 생성 도중 실패했을 때도 내부 catch에서
    // 로그를 정리하고 종료할 수 있도록 바깥에 선언한다.
    std::array<std::thread, 5> threads;

    try
    {
        // 1. 공용 로그 큐와 기록 스레드 1개를 생성하고,
        // app/beam/target/comm/math 로거를 등록한다.
        logging::initialize(
            "logs/radar.log",
            4096);

        auto appLogger = spdlog::get("app");

        appLogger->info(
            "[worker=main] Application starting");

        // 2. 종료 시그널을 등록한다.
        if (
            std::signal(SIGINT, requestStop) == SIG_ERR ||
            std::signal(SIGTERM, requestStop) == SIG_ERR)
        {
            throw std::runtime_error(
                "Cannot install shutdown signal handler");
        }

        // 3. 현재 작업 디렉토리의 .env를 읽는다.
        const app::Config config =
            app::ConfigLoader::load(".env");

        // 4. 채널을 생성한다.
        app::BeamChannel beamChannel;
        app::TargetChannel targetChannel;
        app::CommChannel commChannel;

        // 5. 실제 컨트롤러를 생성한다.
        beam::BeamController beamController(
            config.beamConfig);

        target::TargetController targetController(
            config.targetConfig);

        comm::CommController commController(
            config.commConfig);

        // 통신 워커 실행 전에 서버를 연다.
        commController.openServer();

        appLogger->info(
            "[worker=main] Communication server opened");

        // 6. 업무 워커 5개를 생성한다.
        app::BeamWorker beamWorker(
            beamChannel,
            commChannel,
            beamController);

        app::TargetWorker targetWorker(
            targetChannel,
            beamChannel,
            commChannel,
            targetController);

        app::CommWorker receiveWorker(
            commChannel,
            beamChannel,
            targetChannel,
            commController);

        app::CommWorker simulatorSendWorker(
            commChannel,
            beamChannel,
            targetChannel,
            commController);

        app::CommWorker consoleSendWorker(
            commChannel,
            beamChannel,
            targetChannel,
            commController);

        // 스레드 시작 이후 실패는 내부 catch에서 처리한다.
        // 워커가 참조하는 객체가 먼저 소멸하지 않도록 한다.
        try
        {
            // 7. 워커별 스레드를 실행한다.
            threads[0] = std::thread([&]
            {
                runWorker("BeamWorker", [&]
                {
                    beamWorker.run();
                });
            });

            threads[1] = std::thread([&]
            {
                runWorker("TargetWorker", [&]
                {
                    targetWorker.run();
                });
            });

            threads[2] = std::thread([&]
            {
                runWorker("CommReceive", [&]
                {
                    receiveWorker.runReceive();
                });
            });

            threads[3] = std::thread([&]
            {
                runWorker("CommSendSimulator", [&]
                {
                    simulatorSendWorker.runSendSimulator();
                });
            });

            threads[4] = std::thread([&]
            {
                runWorker("CommSendConsole", [&]
                {
                    consoleSendWorker.runSendConsole();
                });
            });

            appLogger->info(
                "[worker=main] Five worker threads started; "
                "beam tick = 10 ms");

            std::cout
                << "[START] Five worker threads running\n"
                << "[BEAM PERIOD] 10 ms\n"
                << "[LOG FILE] logs/radar.log\n";

            printEndpoint(
                "[SERVER] ",
                config.commConfig.server());

            printEndpoint(
                "[CONSOLE SOURCE] ",
                config.commConfig.operatorConsole());

            printEndpoint(
                "[SIMULATOR SOURCE] ",
                config.commConfig.scenarioSimulator());

            std::cout
                << "Send ON, then attitude, then detection messages.\n"
                << "Press Ctrl+C to drain logs and exit."
                << std::endl;

            // 8. main에서 10ms마다 빔 생성 이벤트를 전달한다.
            const auto started =
                std::chrono::steady_clock::now();

            auto nextTick = started;

            while (!stopRequested)
            {
                std::this_thread::sleep_until(nextTick);

                if (stopRequested)
                {
                    break;
                }

                const auto now =
                    std::chrono::steady_clock::now();

                const auto currentTime =
                    std::chrono::duration_cast<
                        std::chrono::milliseconds>(
                            now - started);

                beamChannel.write(
                    app::NextBeamCommandEvent{currentTime});

                nextTick += BEAM_PERIOD;

                if (nextTick <= now)
                {
                    // 지연된 과거 tick을 한꺼번에 쌓지 않는다.
                    nextTick = now + BEAM_PERIOD;
                }
            }

            // 9. 로그를 정리하고 프로세스를 종료한다.
            appLogger->info(
                "[worker=main] Shutdown requested");

            exitAfterLogging(EXIT_SUCCESS);
        }
        catch (const std::exception& error)
        {
            app::reportWorkerError(
                "main",
                error.what());

            exitAfterLogging(EXIT_FAILURE);
        }
        catch (...)
        {
            app::reportWorkerError(
                "main",
                "Unknown thread or timer failure");

            exitAfterLogging(EXIT_FAILURE);
        }
    }
    catch (const std::exception& error)
    {
        app::reportWorkerError(
            "main",
            error.what());
    }
    catch (...)
    {
        app::reportWorkerError(
            "main",
            "Unknown initialization failure");
    }

    // 업무 스레드 시작 전 초기화에 실패한 경우.
    try
    {
        logging::shutdown();
    }
    catch (const std::exception& error)
    {
        app::reportWorkerError(
            "main",
            error.what());
    }
    catch (...)
    {
        app::reportWorkerError(
            "main",
            "Logging shutdown failed");
    }

    return EXIT_FAILURE;
}