#include <target/controller/target_controller.hpp>

#include "logger/detection_event_csv_logger.hpp"

#include <spdlog/sinks/callback_sink.h>
#include <spdlog/spdlog.h>

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace app
{
namespace
{
using namespace math::literals;
using namespace std::chrono_literals;
using target::DetectionEvent;
using target::TargetController;

target::TargetConfig configuration()
{
    return target::TargetConfig(
        target::AssociationConfig(1000_mps, 30_mps, 11.345),
        target::MeasurementConfig(7_m),
        target::TrackingConfig(10_mps2, 0_mps2),
        target::TrackManagementConfig(100ms, 1000ms, 3, 100)
    );
}

DetectionEvent detection(
    beam::BeamType type,
    std::uint32_t id,
    std::chrono::milliseconds time,
    bool detected = true
) {
    std::vector<DetectionEvent::Plot> plots;
    if (detected)
    {
        plots.emplace_back(
            math::Distance(1200.5_m), math::CircularAngle(12.5_deg), math::TiltAngle(-3.25_deg),
            -20.5_mps, -12.5f
        );
    }
    return DetectionEvent(type, id, time, std::move(plots));
}

class DetectionCsvLoggerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        _logger = std::make_shared<spdlog::logger>(
            DetectionEventCsvLogger::LOGGER_NAME,
            std::make_shared<spdlog::sinks::callback_sink_mt>([this](const spdlog::details::log_msg& message)
            {
                _rows.emplace_back(message.payload.data(), message.payload.size());
            })
        );
        spdlog::register_logger(_logger);
    }

    void TearDown() override
    {
        spdlog::drop(DetectionEventCsvLogger::LOGGER_NAME);
        _logger.reset();
    }

    std::shared_ptr<DetectionEventCsvLogger> _detectionLogger = std::make_shared<DetectionEventCsvLogger>();
    std::shared_ptr<spdlog::logger> _logger;
    std::vector<std::string> _rows;
};

TEST_F(DetectionCsvLoggerTest, ControllerLogsEveryValidatedBeamType)
{
    TargetController controller(configuration(), _detectionLogger);
    EXPECT_TRUE(controller.handleDetection(detection(beam::BeamType::SEARCH, 0, 1000ms)));
    controller.handleDetection(detection(beam::BeamType::CONFIRMATION, 7, 1100ms));
    controller.handleDetection(detection(beam::BeamType::TRACKING, 7, 2100ms));

    ASSERT_EQ(_rows.size(), 3u);
    EXPECT_EQ(_rows[0], "SEARCH,0,1000,1,0,1200.5,12.5,-3.25,-20.5,-12.5");
    EXPECT_EQ(_rows[1], "CONFIRMATION,7,1100,1,0,1200.5,12.5,-3.25,-20.5,-12.5");
    EXPECT_EQ(_rows[2], "TRACKING,7,2100,1,0,1200.5,12.5,-3.25,-20.5,-12.5");
}

TEST_F(DetectionCsvLoggerTest, BatchesPlotsAndPreservesEmptyEvents)
{
    TargetController controller(configuration(), _detectionLogger);
    auto multiple = detection(beam::BeamType::SEARCH, 0, 1000ms);
    multiple.plots.push_back(multiple.plots.front());
    controller.handleDetection(multiple);
    controller.handleDetection(detection(beam::BeamType::TRACKING, 7, 2000ms, false));

    ASSERT_EQ(_rows.size(), 2u);
    EXPECT_EQ(_rows[0],
        "SEARCH,0,1000,2,0,1200.5,12.5,-3.25,-20.5,-12.5\n"
        "SEARCH,0,1000,2,1,1200.5,12.5,-3.25,-20.5,-12.5"
    );
    EXPECT_EQ(_rows[1], "TRACKING,7,2000,0,,,,,,");
}

TEST_F(DetectionCsvLoggerTest, InvalidEventsDoNotProduceRows)
{
    TargetController controller(configuration(), _detectionLogger);
    EXPECT_THROW(
        controller.handleDetection(detection(static_cast<beam::BeamType>(255), 0, 0ms, false)),
        std::invalid_argument
    );
    EXPECT_THROW(
        controller.handleDetection(detection(beam::BeamType::SEARCH, 0, -1ms, false)),
        std::invalid_argument
    );
    EXPECT_THROW(
        controller.handleDetection(detection(beam::BeamType::TRACKING, 0, 0ms, false)),
        std::invalid_argument
    );
    auto invalidPlot = detection(beam::BeamType::SEARCH, 0, 0ms);
    invalidPlot.plots.front().power_db = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(controller.handleDetection(invalidPlot), std::invalid_argument);
    EXPECT_TRUE(_rows.empty());
    controller.handleDetection(detection(beam::BeamType::SEARCH, 0, 1ms, false));
    ASSERT_EQ(_rows.size(), 1u);
    EXPECT_EQ(_rows.front(), "SEARCH,0,1,0,,,,,,");
}

TEST_F(DetectionCsvLoggerTest, RecordsValidatedEventBeforeTrackingFailure)
{
    TargetController controller(configuration(), _detectionLogger);
    const auto time = std::chrono::milliseconds::max();

    EXPECT_THROW(
        controller.handleDetection(detection(beam::BeamType::SEARCH, 0, time)),
        std::overflow_error
    );
    ASSERT_EQ(_rows.size(), 1u);
    EXPECT_EQ(_rows.front(), "SEARCH,0," + std::to_string(time.count()) +
        ",1,0,1200.5,12.5,-3.25,-20.5,-12.5");
}

TEST_F(DetectionCsvLoggerTest, MissingLoggerDoesNotInterruptTracking)
{
    spdlog::drop(DetectionEventCsvLogger::LOGGER_NAME);
    TargetController controller(configuration(), _detectionLogger);

    EXPECT_TRUE(controller.handleDetection(detection(beam::BeamType::SEARCH, 0, 0ms)));
    EXPECT_TRUE(_rows.empty());
}

TEST_F(DetectionCsvLoggerTest, WriterFailureDoesNotInterruptTrackLifecycle)
{
    std::size_t attempts = 0;
    std::vector<std::string> errors;
    _logger->sinks().front() = std::make_shared<spdlog::sinks::callback_sink_mt>(
        [&attempts](const spdlog::details::log_msg&)
        {
            ++attempts;
            throw std::runtime_error("CSV writer unavailable");
        }
    );
    _logger->set_error_handler([&errors](const std::string& error)
    {
        errors.push_back(error);
    });
    TargetController controller(configuration(), _detectionLogger);
    const auto confirmation = controller.handleDetection(detection(beam::BeamType::SEARCH, 0, 0ms));
    ASSERT_TRUE(confirmation);
    const auto tracking = controller.handleDetection(detection(
        beam::BeamType::CONFIRMATION, confirmation->requestId, confirmation->transmitTime
    ));
    ASSERT_TRUE(tracking);
    EXPECT_EQ(controller.nextTrackEvents().size(), 1u);
    EXPECT_EQ(attempts, 2u);
    EXPECT_EQ(errors.size(), 2u);
}

TEST_F(DetectionCsvLoggerTest, ControllerCopiesLogTheirEvents)
{
    TargetController controller(configuration(), _detectionLogger);
    auto copy = controller;
    controller.handleDetection(detection(beam::BeamType::SEARCH, 0, 0ms, false));
    copy.handleDetection(detection(beam::BeamType::SEARCH, 0, 1ms, false));
    EXPECT_EQ(_rows, (std::vector<std::string>{"SEARCH,0,0,0,,,,,,", "SEARCH,0,1,0,,,,,,"}));
}

TEST_F(DetectionCsvLoggerTest, UsesTheCurrentlyRegisteredLogger)
{
    TargetController controller(configuration(), _detectionLogger);
    controller.handleDetection(detection(beam::BeamType::SEARCH, 0, 0ms, false));
    spdlog::drop(DetectionEventCsvLogger::LOGGER_NAME);
    std::vector<std::string> replacementRows;
    spdlog::register_logger(std::make_shared<spdlog::logger>(
        DetectionEventCsvLogger::LOGGER_NAME,
        std::make_shared<spdlog::sinks::callback_sink_mt>([&replacementRows](const spdlog::details::log_msg& message)
        {
            replacementRows.emplace_back(message.payload.data(), message.payload.size());
        })
    ));
    controller.handleDetection(detection(beam::BeamType::SEARCH, 0, 1ms, false));

    EXPECT_EQ(_rows, (std::vector<std::string>{"SEARCH,0,0,0,,,,,,"}));
    EXPECT_EQ(replacementRows, (std::vector<std::string>{"SEARCH,0,1,0,,,,,,"}));
}
}
}
