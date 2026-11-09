#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <app/channel/target_channel.hpp>

#include <app/log/csv_log_manager.hpp>
#include <app/log/message_log_manager.hpp>

#include <app/worker/beam_worker.hpp>
#include <app/worker/comm_worker.hpp>
#include <app/worker/target_worker.hpp>

#include <beam/controller/beam_controller.hpp>
#include <comm/controller/comm_controller.hpp>
#include <target/controller/target_controller.hpp>

#include <spdlog/spdlog.h>

#include "config_loader.hpp"
#include "log/logger/detection_event_csv_logger.hpp"

#include <array>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

namespace
{

constexpr std::chrono::milliseconds BEAM_PERIOD{10};

constexpr char LOG_PATH[] = "logs/adp.log";
constexpr char DETECTION_LOG_PATH[] = "logs/adp_detection.csv";

volatile std::sig_atomic_t stopRequested = 0;

void requestStop(int) noexcept
{
    // 시그널 처리 중에는 종료 플래그만 변경한다.
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

class SpdlogRuntime
{
public:
    SpdlogRuntime()
    {
        // 레지스트리가 로그 관리자보다 나중에 소멸하도록 먼저 생성한다.
        spdlog::default_logger();
    }

    ~SpdlogRuntime() noexcept
    {
        // 로그 관리자 소멸 후 전역 자원을 한 번만 정리한다.
        try
        {
            spdlog::shutdown();
        }
        catch (...)
        {
            std::fprintf(stderr, "[main] Logging shutdown failed\n");
        }
    }
};

template <typename Action>
void runWorker(
    const char* name,
    Action action
) {
    std::shared_ptr<spdlog::logger> logger;

    try
    {
        // 등록된 워커 이름으로 로거를 직접 조회한다.
        logger = spdlog::get(name);

        if (!logger)
        {
            throw std::runtime_error(
                "Worker logger is not registered");
        }

        logger->info("Worker started");

        // 실제 워커의 실행 함수를 호출한다.
        action();

        // 상시 실행하는 워커가 반환되면 예상치 못한 종료로 처리한다.
        logger->error(
            "Worker returned unexpectedly");
    }
    catch (const std::exception& error)
    {
        if (logger)
        {
            logger->error(
                "{}",
                error.what());
        }
        else
        {
            std::fprintf(
                stderr,
                "[%s] %s\n",
                name,
                error.what());
        }
    }
    catch (...)
    {
        if (logger)
        {
            logger->error(
                "Unknown worker failure");
        }
        else
        {
            std::fprintf(
                stderr,
                "[%s] Unknown worker failure\n",
                name);
        }
    }

    // 일반적인 모듈 예외는 각 워커 내부에서 처리한다.
    // 여기까지 도달한 경우는 워커 자체가 종료된 상황이다.
    std::exit(EXIT_FAILURE);
}

} // namespace

int main()
{
    std::array<std::thread, 5> threads;

    std::shared_ptr<spdlog::logger> mainLogger;

    try
    {
        // 1. 생성 시 로그 큐, 기록 스레드와 출력 파일을 준비한다.
        // std::exit()에서도 소멸자가 실행되도록 프로그램 수명으로 유지한다.
        // 선언의 역순으로 CSV, 메시지, 전역 spdlog 자원을 정리한다.
        static SpdlogRuntime spdlogRuntime;
        static app::MessageLogManager messageLogManager(
            LOG_PATH,
            4096
        );
        static app::CsvLogManager csvLogManager(4096);

        // 2. main 스레드용 로거를 등록한다.
        messageLogManager.addLogger("main");
        mainLogger = spdlog::get("main");

        const auto detectionCsvPath = csvLogManager.addTimestampedLogger(
            app::DetectionEventCsvLogger::LOGGER_NAME,
            DETECTION_LOG_PATH,
            app::DetectionEventCsvLogger::header()
        );
        std::cout << "[DETECTION CSV FILE] " << detectionCsvPath.string() << '\n' << std::flush;

        // 3. 업무 워커별 로거를 등록한다.
        // 기본 설정은 INFO 이상 기록, 큐 포화 시 새 로그 버리기다.
        for (const char* name :
             {
                 "BeamWorker",
                 "TargetWorker",
                 "CommReceive",
                 "CommSendSimulator",
                 "CommSendConsole"
             })
        {
            messageLogManager.addLogger(name);
        }

        mainLogger->info(
            "Application starting");

        // 4. 종료 시그널을 등록한다.
        if (
            std::signal(SIGINT, requestStop) == SIG_ERR ||
            std::signal(SIGTERM, requestStop) == SIG_ERR)
        {
            throw std::runtime_error(
                "Cannot install shutdown signal handler");
        }

        // 5. 현재 작업 디렉토리의 설정 파일을 읽는다.
        const app::Config config =
            app::ConfigLoader::load(".env");

        // 6. 채널을 생성한다.
        app::BeamChannel beamChannel;
        app::TargetChannel targetChannel;
        app::CommChannel commChannel;

        // 7. 실제 컨트롤러를 생성한다.
        beam::BeamController beamController(
            config.beamConfig);

        target::TargetController targetController(
            config.targetConfig,
            std::make_shared<app::DetectionEventCsvLogger>()
        );

        comm::CommController commController(
            config.commConfig);

        // 통신 워커를 실행하기 전에 서버를 연다.
        commController.openServer();

        mainLogger->info(
            "Communication server opened");

        // 8. 업무 워커 5개를 생성한다.
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

        // 스레드 시작 이후의 실패는 이 범위에서 처리한다.
        // 워커가 참조하는 채널과 컨트롤러가 먼저 소멸하지 않게 한다.
        try
        {
            // 9. 각 워커를 별도 스레드에서 실행한다.
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

            mainLogger->info(
                "Five worker threads started; beam tick = 10 ms");

            std::cout
                << "[START] Five worker threads running\n"
                << "[BEAM PERIOD] 10 ms\n";

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

            // 10. main에서 10ms마다 빔 생성 이벤트를 전달한다.
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
                    // 지연된 과거 이벤트를 한꺼번에 쌓지 않는다.
                    nextTick = now + BEAM_PERIOD;
                }
            }

            // 11. 로그를 정리하고 프로세스를 종료한다.
            mainLogger->info(
                "Shutdown requested");

            std::exit(EXIT_SUCCESS);
        }
        catch (const std::exception& error)
        {
            mainLogger->error(
                "{}",
                error.what());

            std::exit(EXIT_FAILURE);
        }
        catch (...)
        {
            mainLogger->error(
                "Unknown thread or timer failure");

            std::exit(EXIT_FAILURE);
        }
    }
    catch (const std::exception& error)
    {
        if (mainLogger)
        {
            mainLogger->error(
                "{}",
                error.what());
        }
        else
        {
            // 로거 초기화 전 오류는 콘솔로 출력한다.
            std::fprintf(
                stderr,
                "[main] %s\n",
                error.what());
        }
    }
    catch (...)
    {
        if (mainLogger)
        {
            mainLogger->error(
                "Unknown initialization failure");
        }
        else
        {
            std::fprintf(
                stderr,
                "[main] Unknown initialization failure\n");
        }
    }

    std::exit(EXIT_FAILURE);
}
