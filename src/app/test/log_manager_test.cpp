#include "log/csv_log_manager.hpp"
#include "log/message_log_manager.hpp"

#include <target/controller/target_controller.hpp>

#include "logger/detection_event_csv_logger.hpp"
#include "support/timestamped_log_path.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/callback_sink.h>
#include <spdlog/sinks/null_sink.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace
{
std::atomic<unsigned int> directoryCounter{0};

class FailingRegistrationLogManager final : public app::LogManager
{
public:
    FailingRegistrationLogManager()
      : LogManager(4)
    {
    }

    void addLogger(const std::string& name)
    {
        LogManager::addLogger(
            name,
            spdlog::level::info,
            spdlog::async_overflow_policy::block,
            std::bind(&FailingRegistrationLogManager::configureLogger, this, std::placeholders::_1),
            std::bind(&FailingRegistrationLogManager::onLoggerRegistered, this)
        );
    }

private:
    void configureLogger(spdlog::logger& logger) const
    {
        logger.sinks().push_back(std::make_shared<spdlog::sinks::null_sink_mt>());
    }

    void onLoggerRegistered()
    {
        if (_failNextRegistration)
        {
            _failNextRegistration = false;
            throw std::runtime_error("Registration bookkeeping failed");
        }
    }

    bool _failNextRegistration = true;
};

class LogManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        const auto timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
        _directory = std::filesystem::temp_directory_path() /
            ("aesa_processor_log_test_" + std::to_string(timestamp) + "_" +
             std::to_string(directoryCounter.fetch_add(1)));
        std::filesystem::create_directory(_directory);
        _csvLogs = std::make_unique<app::CsvLogManager>(4);
    }

    void TearDown() override
    {
        _csvLogs.reset();
        _messageLogs.reset();
        spdlog::shutdown();

        std::error_code error;
        for (const auto& entry : std::filesystem::directory_iterator(_directory))
        {
            std::filesystem::remove(entry.path(), error);
        }
        std::filesystem::remove(_directory, error);
    }

    std::filesystem::path path(const std::string& name) const
    {
        return _directory / name;
    }

    std::vector<std::string> lines(const std::string& name) const
    {
        std::ifstream file(path(name));
        std::vector<std::string> result;
        std::string line;
        while (std::getline(file, line))
        {
            result.push_back(line);
        }
        return result;
    }

    std::string messageLogPath(
        const std::string& stem,
        const std::filesystem::path& previous = {}
    ) const {
        const auto prefix = stem + "_";
        for (const auto& entry : std::filesystem::directory_iterator(_directory))
        {
            const auto& filePath = entry.path();
            if (filePath != previous && filePath.extension() == ".log" &&
                filePath.filename().string().rfind(prefix, 0) == 0)
            {
                return filePath.string();
            }
        }
        throw std::runtime_error("Timestamped message log file not found");
    }

    void initializeMessages()
    {
        _messageLogs = std::make_unique<app::MessageLogManager>(
            path("messages.log").string(),
            4
        );
        _messageLogPath = messageLogPath("messages");
        _messageLogs->addLogger("message", spdlog::level::info, spdlog::async_overflow_policy::block);
    }

    std::unique_ptr<app::MessageLogManager> _messageLogs;
    std::unique_ptr<app::CsvLogManager> _csvLogs;
    std::filesystem::path _directory;
    std::string _messageLogPath;
};

TEST_F(LogManagerTest, MessageDestructionDrainsAllLevelsToOneFile)
{
    initializeMessages();
    const auto logger = spdlog::get("message");
    logger->debug("filtered");
    for (int index = 0; index < 200; ++index)
    {
        logger->info("measurement {}", index);
    }
    logger->warn("weak measurement");
    logger->error("failed measurement");
    logger->critical("fatal measurement");

    _messageLogs.reset();

    const auto messages = lines(_messageLogPath);
    ASSERT_EQ(messages.size(), 203u);
    EXPECT_NE(messages.front().find("[message] [info]"), std::string::npos);
    EXPECT_NE(messages[199].find("measurement 199"), std::string::npos);
    EXPECT_NE(messages[200].find("[warning]"), std::string::npos);
    EXPECT_NE(messages[200].find("weak measurement"), std::string::npos);
    EXPECT_NE(messages[201].find("[error]"), std::string::npos);
    EXPECT_NE(messages[201].find("failed measurement"), std::string::npos);
    EXPECT_NE(messages[202].find("[critical]"), std::string::npos);
    EXPECT_NE(messages[202].find("fatal measurement"), std::string::npos);
    EXPECT_FALSE(std::filesystem::exists(path("errors.log")));
    EXPECT_EQ(spdlog::get("message"), nullptr);
}

TEST_F(LogManagerTest, MessageConsoleOutputIsOffByDefaultAndFilteredPerLogger)
{
    initializeMessages();
    _messageLogs->addLogger(
        "console_message",
        spdlog::level::info,
        spdlog::async_overflow_policy::block,
        spdlog::level::warn
    );

    testing::internal::CaptureStdout();
    spdlog::get("message")->warn("file only warning");
    spdlog::get("console_message")->info("file only info");
    spdlog::get("console_message")->warn("console warning");
    _messageLogs.reset();
    const auto console = testing::internal::GetCapturedStdout();

    // 콘솔 레벨을 준 로거의 그 레벨 이상 메시지만 콘솔에 나온다.
    EXPECT_EQ(console.find("file only"), std::string::npos);
    EXPECT_NE(console.find("[console_message] [warning] console warning"), std::string::npos);

    // 파일에는 콘솔 설정과 관계없이 모두 기록된다.
    const auto messages = lines(_messageLogPath);
    ASSERT_EQ(messages.size(), 3u);
    EXPECT_NE(messages[0].find("file only warning"), std::string::npos);
    EXPECT_NE(messages[1].find("file only info"), std::string::npos);
    EXPECT_NE(messages[2].find("console warning"), std::string::npos);
}

TEST_F(LogManagerTest, MessageNamesFilesByStartTimeAndKeepsRunsSeparate)
{
    {
        std::ofstream previousLog(path("messages.log"));
        previousLog << "previous run\n";
    }

    initializeMessages();
    const std::filesystem::path firstLogPath = _messageLogPath;
    EXPECT_EQ(firstLogPath.parent_path(), _directory);
    EXPECT_EQ(firstLogPath.extension(), ".log");
    EXPECT_TRUE(std::regex_match(
        firstLogPath.filename().string(),
        std::regex(R"(messages_[0-9]{8}_[0-9]{6}_[0-9]{3}(_[0-9]+)?\.log)")
    ));

    auto nextLogs = std::make_unique<app::MessageLogManager>(path("messages.log").string(), 4);
    const auto nextLogPath = messageLogPath("messages", firstLogPath);
    EXPECT_NE(nextLogPath, _messageLogPath);
    nextLogs->addLogger("next_message", spdlog::level::info, spdlog::async_overflow_policy::block);
    spdlog::get("message")->info("first run");
    spdlog::get("next_message")->info("next run");
    _messageLogs.reset();
    nextLogs.reset();

    const auto firstMessages = lines(_messageLogPath);
    const auto nextMessages = lines(nextLogPath);
    ASSERT_EQ(firstMessages.size(), 1u);
    ASSERT_EQ(nextMessages.size(), 1u);
    EXPECT_NE(firstMessages.front().find("first run"), std::string::npos);
    EXPECT_NE(nextMessages.front().find("next run"), std::string::npos);
    EXPECT_EQ(lines("messages.log"), (std::vector<std::string>{"previous run"}));
}

TEST_F(LogManagerTest, BaseDestructionDrainsRecordsAndLeavesOtherManagersRunning)
{
    initializeMessages();
    _csvLogs->addLogger("csv", path("detections.csv"), "id,count");
    const auto messageLogger = spdlog::get("message");
    const auto csvLogger = spdlog::get("csv");
    for (int index = 0; index < 100; ++index)
    {
        messageLogger->info("measurement {}", index);
        csvLogger->info("{},0", index);
    }

    std::unique_ptr<app::LogManager> messageManager = std::move(_messageLogs);
    std::unique_ptr<app::LogManager> csvManager = std::move(_csvLogs);
    messageManager.reset();
    EXPECT_EQ(spdlog::get("message"), nullptr);
    ASSERT_EQ(spdlog::get("csv"), csvLogger);
    csvLogger->info("100,0");
    csvManager.reset();

    EXPECT_EQ(lines(_messageLogPath).size(), 100u);
    const auto rows = lines("detections.csv");
    ASSERT_EQ(rows.size(), 102u);
    EXPECT_EQ(rows.front(), "id,count");
    EXPECT_EQ(rows.back(), "100,0");
    EXPECT_EQ(spdlog::get("csv"), nullptr);
}

TEST_F(LogManagerTest, TimestampedPathsPreserveFilesFromTheSameMillisecond)
{
    const auto started = std::chrono::system_clock::time_point(std::chrono::seconds(1700000000));
    const auto firstPath = app::reserveTimestampedLogPath(path("detections.csv"), started);
    {
        std::ofstream file(firstPath);
        file << "previous run\n";
    }
    const auto nextPath = app::reserveTimestampedLogPath(path("detections.csv"), started);

    EXPECT_NE(firstPath, nextPath);
    EXPECT_EQ(nextPath.stem().string(), firstPath.stem().string() + "_1");
    EXPECT_EQ(lines(firstPath.string()), (std::vector<std::string>{"previous run"}));
    EXPECT_TRUE(std::filesystem::exists(nextPath));
    EXPECT_EQ(std::filesystem::file_size(nextPath), 0u);
}

TEST_F(LogManagerTest, TimestampedPathsAreDistinctForConcurrentRuns)
{
    constexpr std::size_t RUN_COUNT = 16;
    const auto started = std::chrono::system_clock::time_point(std::chrono::seconds(1700000000));
    std::promise<void> start;
    const auto ready = start.get_future().share();
    std::vector<std::filesystem::path> paths(RUN_COUNT);
    std::vector<std::exception_ptr> errors(RUN_COUNT);
    std::vector<std::thread> runs;
    for (std::size_t index = 0; index < RUN_COUNT; ++index)
    {
        runs.emplace_back([&, index]
        {
            ready.wait();
            try
            {
                paths[index] = app::reserveTimestampedLogPath(path("detections.csv"), started);
            }
            catch (...)
            {
                errors[index] = std::current_exception();
            }
        });
    }
    start.set_value();
    for (auto& run : runs)
    {
        run.join();
    }
    for (const auto& error : errors)
    {
        if (error)
        {
            std::rethrow_exception(error);
        }
    }

    const std::set<std::filesystem::path> uniquePaths(paths.begin(), paths.end());
    EXPECT_EQ(uniquePaths.size(), RUN_COUNT);
    for (const auto& outputPath : paths)
    {
        EXPECT_TRUE(std::filesystem::exists(outputPath));
        EXPECT_EQ(std::filesystem::file_size(outputPath), 0u);
    }
}

TEST_F(LogManagerTest, FailedTimestampedCsvRegistrationRemovesReservedFiles)
{
    _csvLogs->addLogger("csv", path("detections.csv"), "id,count");
    EXPECT_THROW(
        _csvLogs->addTimestampedLogger("csv", path("rejected.csv"), "id,count"),
        std::invalid_argument
    );
    EXPECT_THROW(
        _csvLogs->addTimestampedLogger("new_csv", path("rejected.csv"), ""),
        std::invalid_argument
    );
    EXPECT_THROW(
        _csvLogs->addTimestampedLogger("", path("rejected.csv"), "id,count"),
        std::invalid_argument
    );
    EXPECT_EQ(spdlog::get("new_csv"), nullptr);
    std::set<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(_directory))
    {
        files.insert(entry.path().filename());
    }
    EXPECT_EQ(files, (std::set<std::filesystem::path>{"detections.csv"}));
}

TEST_F(LogManagerTest, DuplicateRegistrationKeepsOriginalLogger)
{
    initializeMessages();
    const auto originalLogger = spdlog::get("message");
    EXPECT_THROW(_messageLogs->addLogger("message"), spdlog::spdlog_ex);
    ASSERT_EQ(spdlog::get("message"), originalLogger);
    originalLogger->info("still running");
    _messageLogs.reset();

    const auto messages = lines(_messageLogPath);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_NE(messages.front().find("still running"), std::string::npos);
    EXPECT_EQ(spdlog::get("message"), nullptr);
}

TEST_F(LogManagerTest, FailedRegistrationHookRollsBackAndAllowsRetry)
{
    {
        FailingRegistrationLogManager manager;
        EXPECT_THROW(manager.addLogger("retry_logger"), std::runtime_error);
        EXPECT_EQ(spdlog::get("retry_logger"), nullptr);
        EXPECT_NO_THROW(manager.addLogger("retry_logger"));
        ASSERT_NE(spdlog::get("retry_logger"), nullptr);
        spdlog::get("retry_logger")->info("registered after retry");
    }
    EXPECT_EQ(spdlog::get("retry_logger"), nullptr);
}

TEST_F(LogManagerTest, CsvWritesOnlyHeaderAndRowsToSeparateFiles)
{
    _csvLogs->addLogger("detection_csv", path("detections.csv"), "requestId,count");
    _csvLogs->addLogger("track_csv", path("tracks.csv"), "trackId,range_m");
    spdlog::get("detection_csv")->info("{},{}", 7, 0);
    spdlog::get("track_csv")->info("{},{}", 3, 1500);
    _csvLogs.reset();

    EXPECT_EQ(lines("detections.csv"), (std::vector<std::string>{"requestId,count", "7,0"}));
    EXPECT_EQ(lines("tracks.csv"), (std::vector<std::string>{"trackId,range_m", "3,1500"}));
}

TEST_F(LogManagerTest, CsvAppendsWithoutRepeatingHeader)
{
    _csvLogs->addLogger("first_csv", path("detections.csv"), "id,count");
    spdlog::get("first_csv")->info("1,2");
    _csvLogs.reset();

    {
        app::CsvLogManager nextLogs(4);
        nextLogs.addLogger("next_csv", path("detections.csv"), "id,count");
        spdlog::get("next_csv")->info("2,0");
    }

    EXPECT_EQ(lines("detections.csv"), (std::vector<std::string>{"id,count", "1,2", "2,0"}));
}

TEST_F(LogManagerTest, CsvWritesBatchedRowsWithoutAddingMetadata)
{
    _csvLogs->addLogger("csv", path("detections.csv"), "id,count");
    const auto logger = spdlog::get("csv");
    const auto loggedAt = spdlog::log_clock::time_point(std::chrono::milliseconds(1234));
    logger->log(loggedAt, spdlog::source_loc{}, spdlog::level::info, "1,2\n2,0");
    logger->log(loggedAt, spdlog::source_loc{}, spdlog::level::info, "3,1\n4,0");
    _csvLogs.reset();

    EXPECT_EQ(lines("detections.csv"), (std::vector<std::string>{
        "id,count",
        "1,2",
        "2,0",
        "3,1",
        "4,0"
    }));
}

TEST_F(LogManagerTest, CsvRejectsDifferentHeaderAndIncompleteLastRow)
{
    {
        std::ofstream file(path("detections.csv"));
        file << "id,count\n1,2\n";
    }
    EXPECT_THROW(
        _csvLogs->addLogger("csv", path("detections.csv"), "id,range"),
        std::invalid_argument
    );
    EXPECT_EQ(spdlog::get("csv"), nullptr);
    EXPECT_EQ(lines("detections.csv"), (std::vector<std::string>{"id,count", "1,2"}));

    {
        std::ofstream file(path("incomplete.csv"));
        file << "id,count\n1,2";
    }
    EXPECT_THROW(
        _csvLogs->addLogger("csv", path("incomplete.csv"), "id,count"),
        std::invalid_argument
    );
}

TEST_F(LogManagerTest, MessageDestructionLeavesOtherManagersRunning)
{
    initializeMessages();
    auto otherLogs = std::make_unique<app::MessageLogManager>(
        path("other.log").string(),
        4
    );
    const auto otherLogPath = messageLogPath("other");
    otherLogs->addLogger("other_message");
    _csvLogs->addLogger("csv", path("detections.csv"), "id,count");

    _messageLogs.reset();
    ASSERT_NE(spdlog::get("other_message"), nullptr);
    ASSERT_NE(spdlog::get("csv"), nullptr);
    spdlog::get("other_message")->info("still running");
    spdlog::get("csv")->info("1,0");
    otherLogs.reset();
    _csvLogs.reset();

    const auto messages = lines(otherLogPath);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_NE(messages.front().find("still running"), std::string::npos);
    EXPECT_EQ(lines("detections.csv"), (std::vector<std::string>{"id,count", "1,0"}));
}

TEST_F(LogManagerTest, CsvDestructionLeavesMessageLoggingRunning)
{
    initializeMessages();
    _csvLogs->addLogger("csv", path("detections.csv"), "id,count");
    _csvLogs.reset();
    EXPECT_EQ(spdlog::get("csv"), nullptr);
    ASSERT_NE(spdlog::get("message"), nullptr);
    spdlog::get("message")->info("still running");
    _messageLogs.reset();

    const auto messages = lines(_messageLogPath);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_NE(messages.front().find("still running"), std::string::npos);
}

TEST_F(LogManagerTest, CsvRejectsDuplicateNamesAndFileAliasesWithoutChangingData)
{
    initializeMessages();
    EXPECT_THROW(
        _csvLogs->addLogger("message", path("detections.csv"), "id,count"),
        std::invalid_argument
    );
    EXPECT_FALSE(std::filesystem::exists(path("detections.csv")));

    _csvLogs->addLogger("csv", path("detections.csv"), "id,count");
    spdlog::get("csv")->info("1,0");
    EXPECT_THROW(
        _csvLogs->addLogger("csv", path("other.csv"), "id,count"),
        std::invalid_argument
    );
    EXPECT_FALSE(std::filesystem::exists(path("other.csv")));
    EXPECT_THROW(
        _csvLogs->addLogger("alias", _directory / "." / "detections.csv", "id,count"),
        std::invalid_argument
    );
    _csvLogs.reset();
    EXPECT_EQ(lines("detections.csv"), (std::vector<std::string>{"id,count", "1,0"}));
}

TEST_F(LogManagerTest, TargetControllerWritesTimestampedDetectionCsv)
{
    using namespace math::literals;
    using namespace std::chrono_literals;
    const auto outputPath = _csvLogs->addTimestampedLogger(
        app::DetectionEventCsvLogger::LOGGER_NAME,
        path("adp_detection.csv"),
        app::DetectionEventCsvLogger::header()
    );
    EXPECT_EQ(outputPath.parent_path(), _directory);
    EXPECT_TRUE(std::regex_match(
        outputPath.filename().string(),
        std::regex(R"(adp_detection_[0-9]{8}_[0-9]{6}_[0-9]{3}(_[0-9]+)?\.csv)")
    ));
    target::TargetController controller(
        target::TargetConfig(
            target::AssociationConfig(1000_mps, 30_mps, 11.345),
            target::MeasurementConfig(7_m),
            target::TrackingConfig(10_mps2, 0_mps2),
            target::TrackManagementConfig(100ms, 1000ms, 3, 100)
        ),
        std::make_shared<app::DetectionEventCsvLogger>()
    );
    controller.handleDetection(target::DetectionEvent(beam::BeamType::SEARCH, 0, 1000ms, {
        {math::Distance(1500_m), math::CircularAngle(30_deg), math::TiltAngle(5_deg), -20_mps, -12.5f},
        {math::Distance(1800_m), math::CircularAngle(32_deg), math::TiltAngle(6_deg), -15_mps, -10.0f}
    }));
    controller.handleDetection(target::DetectionEvent(beam::BeamType::TRACKING, 7, 2000ms, {}));
    _csvLogs.reset();

    EXPECT_EQ(lines(outputPath.string()), (std::vector<std::string>{
        "beamType,requestId,transmitTime_ms,plotCount,"
        "plotIndex,slantRange_m,azimuth_deg,elevation_deg,dopplerVelocity_mps,power_db",
        "SEARCH,0,1000,2,0,1500,30,5,-20,-12.5",
        "SEARCH,0,1000,2,1,1800,32,6,-15,-10",
        "TRACKING,7,2000,0,,,,,,"
    }));
}

TEST_F(LogManagerTest, CsvRecordsOnBackgroundThread)
{
    _csvLogs->addLogger("csv", path("detections.csv"), "id,count");
    const auto logger = spdlog::get("csv");
    const auto callerThread = std::this_thread::get_id();
    std::thread::id writerThread;
    logger->sinks().push_back(std::make_shared<spdlog::sinks::callback_sink_mt>(
        [&writerThread](const spdlog::details::log_msg&)
        {
            writerThread = std::this_thread::get_id();
        }
    ));

    logger->info("1,0");
    _csvLogs.reset();

    EXPECT_NE(writerThread, std::thread::id{});
    EXPECT_NE(writerThread, callerThread);
    EXPECT_EQ(lines("detections.csv"), (std::vector<std::string>{"id,count", "1,0"}));
}

TEST_F(LogManagerTest, CsvPreservesRowOrderWhenQueueIsSmallerThanInput)
{
    _csvLogs->addLogger("csv", path("detections.csv"), "id,count");
    const auto logger = spdlog::get("csv");
    for (int index = 0; index < 1000; ++index)
    {
        logger->info("{},0", index);
    }
    _csvLogs.reset();

    const auto rows = lines("detections.csv");
    ASSERT_EQ(rows.size(), 1001u);
    EXPECT_EQ(rows.front(), "id,count");
    for (int index = 0; index < 1000; ++index)
    {
        EXPECT_EQ(rows[index + 1], std::to_string(index) + ",0");
    }
}

TEST_F(LogManagerTest, CsvKeepsEveryRowFromConcurrentProducers)
{
    _csvLogs->addLogger("csv", path("detections.csv"), "producer,index");
    const auto logger = spdlog::get("csv");
    std::vector<std::thread> producers;
    for (int producer = 0; producer < 4; ++producer)
    {
        producers.emplace_back([logger, producer]
        {
            for (int index = 0; index < 100; ++index)
            {
                logger->info("{},{}", producer, index);
            }
        });
    }
    for (auto& producer : producers)
    {
        producer.join();
    }
    _csvLogs.reset();

    const auto rows = lines("detections.csv");
    ASSERT_EQ(rows.size(), 401u);
    EXPECT_EQ(rows.front(), "producer,index");
    const std::set<std::string> uniqueRows(rows.begin() + 1, rows.end());
    EXPECT_EQ(uniqueRows.size(), 400u);
    for (int producer = 0; producer < 4; ++producer)
    {
        for (int index = 0; index < 100; ++index)
        {
            EXPECT_EQ(uniqueRows.count(std::to_string(producer) + "," + std::to_string(index)), 1u);
        }
    }
}

TEST_F(LogManagerTest, RejectsInvalidConstructorAndCsvConfiguration)
{
    EXPECT_THROW(app::CsvLogManager(0), std::invalid_argument);
    EXPECT_THROW(
        app::MessageLogManager("", 4),
        std::invalid_argument
    );
    EXPECT_THROW(
        app::MessageLogManager(path("messages.log").string(), 0),
        std::invalid_argument
    );
    EXPECT_THROW(_csvLogs->addLogger("", path("data.csv"), "id"), std::invalid_argument);
    EXPECT_THROW(_csvLogs->addLogger("csv", {}, "id"), std::invalid_argument);
    EXPECT_THROW(_csvLogs->addLogger("csv", path("data.csv"), ""), std::invalid_argument);
    EXPECT_THROW(_csvLogs->addLogger("csv", path("data.csv"), "id\ncount"), std::invalid_argument);
    EXPECT_THROW(_csvLogs->addLogger("csv", _directory, "id"), std::filesystem::filesystem_error);
}
}
