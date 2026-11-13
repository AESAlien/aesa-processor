#include "dto/beam_input.hpp"
#include "queue/message_queue.hpp"
#include "dto/target_input.hpp"
#include "dto/console_input.hpp"
#include "dto/simulator_input.hpp"

#include "log/csv_log_manager.hpp"
#include "log/message_log_manager.hpp"

#include "worker/beam_worker.hpp"
#include "worker/comm_receive_worker.hpp"
#include "worker/simulator_send_worker.hpp"
#include "worker/console_send_worker.hpp"
#include "worker/target_worker.hpp"
#include "worker/tick_worker.hpp"

#include <beam/controller/beam_controller.hpp>
#include <comm/controller/comm_controller.hpp>
#include <target/controller/target_controller.hpp>

#include <spdlog/spdlog.h>

#include "config_loader.hpp"
#include "log/logger/beam_event_message_logger.hpp"
#include "log/logger/detection_event_csv_logger.hpp"

#include <array>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <stdexcept>
#include <thread>

namespace
{

constexpr char LOG_PATH[] = "logs/adp.log";
constexpr char DETECTION_LOG_PATH[] = "logs/adp_detection.csv";

volatile std::sig_atomic_t stopRequested = 0;

void requestStop(int) noexcept
{
    // 시그널 처리 중에는 종료 플래그만 변경한다.
    stopRequested = 1;
}

void registerShutdownSignalHandlers()
{
    if (std::signal(SIGINT, requestStop) == SIG_ERR ||
        std::signal(SIGTERM, requestStop) == SIG_ERR)
    {
        throw std::runtime_error("Cannot install shutdown signal handler");
    }
}

void waitForShutdownRequest()
{
    while (!stopRequested)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
}

template <typename WorkerType>
void startWorker(
    std::thread& thread,
    WorkerType& worker
) {
    thread = std::thread([&worker]
    {
        worker.run();
        // 워커가 종료되면 현재 종료 정책에 따라 프로세스를 종료한다.
        std::exit(EXIT_FAILURE);
    });
}

} // namespace

int main()
{
    std::array<std::thread, 6> threads;
    std::shared_ptr<spdlog::logger> mainLogger;

    try
    {
        // 매니저보다 로거 레지스트리를 먼저 생성해 매니저보다 나중에 소멸시킨다.
        spdlog::default_logger();
    }
    catch (...)
    {
        std::fprintf(stderr, "[main] Logging initialization failed\n");
        return EXIT_FAILURE;
    }

    static std::unique_ptr<app::MessageLogManager> messageLogManager;
    static std::unique_ptr<app::CsvLogManager> csvLogManager;

    try
    {
        // 1. 로깅 매니저와 main 로거를 초기화한다.
        messageLogManager = std::make_unique<app::MessageLogManager>(LOG_PATH, 4096);
        csvLogManager = std::make_unique<app::CsvLogManager>(4096);
        mainLogger = messageLogManager->addLogger("main");
    }
    catch (...)
    {
        // 로거 준비 전에는 표준 오류만 사용할 수 있다.
        std::fprintf(stderr, "[main] Logging initialization failed\n");
        return EXIT_FAILURE;
    }

    mainLogger->info("[1/12] Logging managers initialized");

    try
    {
        // 2. 메시지 로거를 등록한다.
        const auto tickLogger = messageLogManager->addLogger("tick");
        const auto beamLogger = messageLogManager->addLogger("beam");
        const auto targetLogger = messageLogManager->addLogger("target");
        const auto commLogger = messageLogManager->addLogger("comm");

        // 빔 처리 이벤트는 메시지 로그와 콘솔에 기록한다.
        messageLogManager->addLogger(
            app::BeamEventMessageLogger::LOGGER_NAME,
            spdlog::level::info,
            spdlog::async_overflow_policy::discard_new,
            spdlog::level::info
        );
        mainLogger->info("[2/12] Message loggers registered");

        // 3. CSV 로거를 등록한다.
        const auto detectionCsvPath = csvLogManager->addTimestampedLogger(
            app::DetectionEventCsvLogger::LOGGER_NAME,
            DETECTION_LOG_PATH,
            app::DetectionEventCsvLogger::header()
        );
        mainLogger->info("[3/12] CSV logger registered: {}", detectionCsvPath.string());

        // 4. 종료 시그널을 처리할 핸들러를 등록한다.
        registerShutdownSignalHandlers();
        mainLogger->info("[4/12] Shutdown signal handlers installed");

        // 5. 애플리케이션 설정을 읽는다.
        const app::Config config = app::ConfigLoader::load(".env");
        mainLogger->info("[5/12] Application configuration loaded");

        // 6. 워커 간 입력 큐를 생성한다.
        app::MessageQueue<app::BeamInput> beamQueue;
        app::MessageQueue<app::TargetInput> targetQueue;
        app::MessageQueue<app::ConsoleInput> consoleQueue;
        app::MessageQueue<app::SimulatorInput> simulatorQueue;
        mainLogger->info("[6/12] Four worker input queues initialized");

        // 7. 설정과 로거를 주입해 컨트롤러를 생성한다.
        beam::BeamController beamController(
            config.beamConfig,
            std::make_shared<app::BeamEventMessageLogger>(),
            beamLogger
        );
        target::TargetController targetController(
            config.targetConfig,
            std::make_shared<app::DetectionEventCsvLogger>(),
            targetLogger
        );
        comm::CommController commController(
            config.commConfig,
            commLogger
        );
        mainLogger->info("[7/12] Controllers initialized");

        // 8. 컨트롤러와 입력 큐를 연결하는 워커를 생성한다.
        app::TickWorker tickWorker(
            beamQueue,
            targetQueue,
            tickLogger
        );
        app::BeamWorker beamWorker(
            beamController,
            beamQueue,
            simulatorQueue,
            beamLogger
        );
        app::TargetWorker targetWorker(
            targetController,
            targetQueue,
            beamQueue,
            consoleQueue,
            targetLogger
        );
        app::CommReceiveWorker commReceiveWorker(
            commController,
            beamQueue,
            targetQueue,
            consoleQueue,
            commLogger
        );
        app::SimulatorSendWorker simulatorSendWorker(
            commController,
            simulatorQueue,
            commLogger
        );
        app::ConsoleSendWorker consoleSendWorker(
            commController,
            consoleQueue,
            commLogger
        );
        mainLogger->info("[8/12] Workers initialized");

        // 9. 수신 워커를 시작하기 전에 통신 서버를 연다.
        commController.openServer();
        mainLogger->info("[9/12] Communication server opened");

        // 10. 각 워커를 별도 스레드에서 실행한다.
        startWorker(threads[0], tickWorker);
        startWorker(threads[1], beamWorker);
        startWorker(threads[2], targetWorker);
        startWorker(threads[3], commReceiveWorker);
        startWorker(threads[4], simulatorSendWorker);
        startWorker(threads[5], consoleSendWorker);
        mainLogger->info("[10/12] All 6 worker threads started");

        // 11. 종료 시그널이 들어올 때까지 대기한다.
        mainLogger->info("[11/12] Application is running; waiting for shutdown signal");
        waitForShutdownRequest();

        // 12. 종료를 기록하고 프로세스를 끝낸다.
        mainLogger->info("[12/12] Shutdown signal received");

        std::exit(EXIT_SUCCESS);
    }
    catch (const std::exception& error)
    {
        mainLogger->error("Application failed: {}", error.what());
    }
    catch (...)
    {
        mainLogger->error("Application failed: unknown error");
    }

    std::exit(EXIT_FAILURE);
}
