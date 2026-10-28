#include "config_loader.hpp"

#include <beam/controller/beam_controller.hpp>
#include <beam/domain/operation_state.hpp>
#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <comm/controller/comm_controller.hpp>
#include <target/controller/target_controller.hpp>
#include <target/dto/detection_event.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>

namespace
{

static_assert(std::is_copy_assignable_v<app::Config>);

using namespace math::literals;
using namespace std::chrono_literals;

std::atomic<unsigned int> fileCounter{0};

std::filesystem::path temporaryPath()
{
    const auto timestamp = std::chrono::steady_clock::now()
        .time_since_epoch()
        .count();
    return std::filesystem::temp_directory_path() /
        ("aesa_processor_config_test_" +
         std::to_string(timestamp) +
         "_" +
         std::to_string(fileCounter.fetch_add(1)) +
         ".env");
}

app::Config loadConfig(const std::filesystem::path& path)
{
    return app::ConfigLoader::load(path);
}

class TemporaryEnvFile
{
public:
    explicit TemporaryEnvFile(const std::string& content)
        : _path(temporaryPath())
    {
        std::ofstream file(_path);
        file << content;
        if (!file)
        {
            throw std::runtime_error("Failed to write temporary .env file");
        }
    }

    ~TemporaryEnvFile()
    {
        std::error_code error;
        std::filesystem::remove(_path, error);
    }

    const std::filesystem::path& path() const noexcept
    {
        return _path;
    }

private:
    std::filesystem::path _path;
};

std::string validConfigText()
{
    return
        "# Non-default values verify every mapping.\n"
        "BEAM_MAX_ABS_PITCH_DEG=60\n"
        "BEAM_MOUNT_ANT_TO_BODY_ROTATION=1, 0, 0, 0, 1, 0, 0, 0, 1\n"
        "MOUNT_ORTHOGONAL_ABSOLUTE_TOLERANCE=0.00001\n"
        "MOUNT_ORTHOGONAL_RELATIVE_TOLERANCE=0.00002\n"
        "BEAM_SEARCH_AZIMUTH_COUNT=3\n"
        "BEAM_SEARCH_ELEVATION_COUNT=2\n"
        "BEAM_SEARCH_AZIMUTH_START_DEG=-10\n"
        "BEAM_SEARCH_AZIMUTH_STEP_DEG=5\n"
        "BEAM_SEARCH_ELEVATION_START_DEG=1\n"
        "BEAM_SEARCH_ELEVATION_STEP_DEG=2\n"
        "BEAM_SEARCH_WIDTH_DEG=4\n"
        "BEAM_CONFIRMATION_WIDTH_DEG=2\n"
        "BEAM_TRACKING_WIDTH_DEG=5\n"
        "TARGET_MAX_ASSOCIATION_SPEED_MPS=900\n"
        "TARGET_MAX_INITIAL_DOPPLER_DIFFERENCE_MPS=25\n"
        "TARGET_MEASUREMENT_STANDARD_DEVIATION_M=6\n"
        "TARGET_POSITION_NDS_THRESHOLD=10\n"
        "TARGET_CONFIRMATION_DELAY_MS=150\n"
        "TARGET_TRACKING_INTERVAL_MS=800\n"
        "TARGET_MAX_CONSECUTIVE_TRACKING_MISSES=4\n"
        "TARGET_INITIAL_ACCELERATION_STANDARD_DEVIATION_MPS2=8\n"
        "TARGET_MAX_REPORTED_TRACK_COUNT=75\n"
        "COMM_SERVER_ADDRESS=127.0.0.1\n"
        "COMM_SERVER_PORT=43200\n"
        "COMM_OPERATOR_CONSOLE_ADDRESS=127.0.0.2\n"
        "COMM_OPERATOR_CONSOLE_PORT=43201\n"
        "COMM_SCENARIO_SIMULATOR_ADDRESS=127.0.0.3\n"
        "COMM_SCENARIO_SIMULATOR_PORT=43202\n";
}

std::string replaceValue(
    std::string content,
    const std::string& key,
    const std::string& value)
{
    const std::string prefix = key + '=';
    const std::size_t start = content.find(prefix);
    if (start == std::string::npos)
    {
        throw std::invalid_argument("Configuration key not found in test input");
    }

    const std::size_t valueStart = start + prefix.size();
    const std::size_t lineEnd = content.find('\n', valueStart);
    content.replace(
        valueStart,
        lineEnd == std::string::npos
            ? std::string::npos
            : lineEnd - valueStart,
        value);
    return content;
}

std::string removeKey(
    std::string content,
    const std::string& key)
{
    const std::string prefix = key + '=';
    const std::size_t start = content.find(prefix);
    if (start == std::string::npos)
    {
        throw std::invalid_argument("Configuration key not found in test input");
    }

    const std::size_t lineEnd = content.find('\n', start);
    content.erase(
        start,
        lineEnd == std::string::npos
            ? std::string::npos
            : lineEnd - start + 1);
    return content;
}

TEST(AppConfigTest, LoadsEveryConfigurationValue)
{
    const TemporaryEnvFile file(validConfigText());
    const app::Config config = loadConfig(file.path());

    EXPECT_EQ(config.beamConfig.attitude.maxAbsPitch(), 60_deg);
    EXPECT_DOUBLE_EQ(
        config.beamConfig.attitude.mountOrthogonalTolerance().absolute(),
        0.00001);
    EXPECT_DOUBLE_EQ(
        config.beamConfig.attitude.mountOrthogonalTolerance().relative(),
        0.00002);
    EXPECT_EQ(
        config.beamConfig.attitude.mountAntToBodyRotation(),
        math::SquareMatrix::identity(3));
    EXPECT_EQ(config.beamConfig.searchGrid.azimuthCount(), 3u);
    EXPECT_EQ(config.beamConfig.searchGrid.elevationCount(), 2u);
    EXPECT_TRUE(config.beamConfig.searchGrid.azimuthStart().isEquivalentTo(-10_deg));
    EXPECT_EQ(config.beamConfig.searchGrid.azimuthStep(), 5_deg);
    EXPECT_EQ(config.beamConfig.searchGrid.elevationStart(), 1_deg);
    EXPECT_EQ(config.beamConfig.searchGrid.elevationStep(), 2_deg);
    EXPECT_EQ(config.beamConfig.searchGrid.beamWidth().azimuth, 4_deg);
    EXPECT_EQ(config.beamConfig.searchGrid.beamWidth().elevation, 4_deg);
    EXPECT_EQ(config.beamConfig.requestBeam.confirmationBeamWidth.azimuth, 2_deg);
    EXPECT_EQ(config.beamConfig.requestBeam.confirmationBeamWidth.elevation, 2_deg);
    EXPECT_EQ(config.beamConfig.requestBeam.trackingBeamWidth.azimuth, 5_deg);
    EXPECT_EQ(config.beamConfig.requestBeam.trackingBeamWidth.elevation, 5_deg);

    EXPECT_EQ(config.targetConfig.maxAssociationSpeed(), 900_mps);
    EXPECT_EQ(config.targetConfig.maxInitialDopplerDifference(), 25_mps);
    EXPECT_EQ(config.targetConfig.measurementStandardDeviation(), 6_m);
    EXPECT_DOUBLE_EQ(config.targetConfig.positionNdsThreshold(), 10.0);
    EXPECT_EQ(config.targetConfig.confirmationDelay(), 150ms);
    EXPECT_EQ(config.targetConfig.trackingInterval(), 800ms);
    EXPECT_EQ(config.targetConfig.maxConsecutiveTrackingMisses(), 4u);
    EXPECT_DOUBLE_EQ(
        config.targetConfig.initialAccelerationStandardDeviation(),
        8.0);
    EXPECT_EQ(config.targetConfig.maxReportedTrackCount(), 75u);

    EXPECT_EQ(config.commConfig.server().address(), "127.0.0.1");
    EXPECT_EQ(config.commConfig.server().port(), 43200);
    EXPECT_EQ(config.commConfig.operatorConsole().address(), "127.0.0.2");
    EXPECT_EQ(config.commConfig.operatorConsole().port(), 43201);
    EXPECT_EQ(config.commConfig.scenarioSimulator().address(), "127.0.0.3");
    EXPECT_EQ(config.commConfig.scenarioSimulator().port(), 43202);
}

TEST(AppConfigTest, RejectsDisabledCommunicationPeer)
{
    std::string content = replaceValue(
        validConfigText(),
        "COMM_OPERATOR_CONSOLE_ADDRESS",
        "");
    content = replaceValue(
        std::move(content),
        "COMM_OPERATOR_CONSOLE_PORT",
        "0");

    const TemporaryEnvFile file(content);
    EXPECT_THROW(loadConfig(file.path()), std::invalid_argument);
}

TEST(AppConfigTest, ExampleConfigurationLoads)
{
    EXPECT_NO_THROW(loadConfig(APP_ENV_EXAMPLE_PATH));
}

TEST(AppConfigTest, LoadsConfigurationFromEnvFilePath)
{
    const TemporaryEnvFile file(validConfigText());
    const app::Config config = app::ConfigLoader::load(file.path());

    EXPECT_EQ(config.beamConfig.searchGrid.azimuthCount(), 3u);
    EXPECT_EQ(config.targetConfig.maxReportedTrackCount(), 75u);
    EXPECT_EQ(config.commConfig.server().port(), 43200);
}

TEST(AppConfigTest, MissingRelativeMountOrthogonalToleranceThrows)
{
    const std::string content = removeKey(
        validConfigText(),
        "MOUNT_ORTHOGONAL_RELATIVE_TOLERANCE");
    const TemporaryEnvFile file(content);

    EXPECT_THROW(loadConfig(file.path()), std::runtime_error);
}

TEST(AppConfigTest, ParsesQuotedValuesAndInlineComments)
{
    std::string content = replaceValue(
        validConfigText(),
        "BEAM_MAX_ABS_PITCH_DEG",
        "\"60\"");
    content = replaceValue(
        content,
        "TARGET_MAX_ASSOCIATION_SPEED_MPS",
        "900 # inline comment");
    content = replaceValue(
        content,
        "COMM_SERVER_ADDRESS",
        "'127.0.0.1'");

    const TemporaryEnvFile file(content);
    const app::Config config = loadConfig(file.path());

    EXPECT_EQ(config.beamConfig.attitude.maxAbsPitch(), 60_deg);
    EXPECT_EQ(config.targetConfig.maxAssociationSpeed(), 900_mps);
    EXPECT_EQ(config.commConfig.server().address(), "127.0.0.1");
}

TEST(AppConfigTest, LoadedConfigurationCanBeInjectedIntoEveryController)
{
    const TemporaryEnvFile file(validConfigText());
    const app::Config config = loadConfig(file.path());

    beam::BeamController beamController(config.beamConfig);
    beamController.setOperationState(
        beam::SetOperationStateCommand(beam::OperationState::ON));
    beamController.setAttitude(beam::SetAttitudeCommand(
        37_deg,
        127_deg,
        0_m,
        0_deg,
        0_deg,
        0_deg));
    const auto beamCommand = beamController.nextBeamCommand(0ms);
    ASSERT_TRUE(beamCommand.has_value());
    EXPECT_EQ(beamCommand->beamWidth.azimuth, 4_deg);
    EXPECT_EQ(beamCommand->beamWidth.elevation, 4_deg);

    target::TargetController targetController(config.targetConfig);
    const auto confirmation = targetController.handleDetection(
        target::DetectionEvent(
            beam::BeamType::SEARCH,
            1,
            1000ms,
            {{math::Distance(1000_m), math::Azimuth(0_deg), math::Elevation(0_deg), 0_mps, -10.0f}}));
    ASSERT_TRUE(confirmation.has_value());
    EXPECT_EQ(confirmation->transmitTime, 1150ms);

    EXPECT_NO_THROW(comm::CommController(config.commConfig));
}

TEST(AppConfigTest, MissingFileOrRequiredKeyThrows)
{
    const std::filesystem::path missingPath = temporaryPath();
    EXPECT_THROW(
        loadConfig(missingPath),
        std::runtime_error);

    const TemporaryEnvFile file(removeKey(
        validConfigText(),
        "TARGET_TRACKING_INTERVAL_MS"));
    EXPECT_THROW(
        loadConfig(file.path()),
        std::runtime_error);
}

TEST(AppConfigTest, InvalidScalarAndDomainValuesThrow)
{
    const TemporaryEnvFile invalidNumber(replaceValue(
        validConfigText(),
        "TARGET_CONFIRMATION_DELAY_MS",
        "100ms"));
    EXPECT_THROW(
        loadConfig(invalidNumber.path()),
        std::runtime_error);

    const TemporaryEnvFile invalidGrid(replaceValue(
        validConfigText(),
        "BEAM_SEARCH_AZIMUTH_COUNT",
        "0"));
    EXPECT_THROW(
        loadConfig(invalidGrid.path()),
        std::runtime_error);

    const TemporaryEnvFile invalidTarget(replaceValue(
        validConfigText(),
        "TARGET_MEASUREMENT_STANDARD_DEVIATION_M",
        "0"));
    EXPECT_THROW(
        loadConfig(invalidTarget.path()),
        std::runtime_error);
}

TEST(AppConfigTest, InvalidCommunicationEndpointsThrow)
{
    const TemporaryEnvFile invalidAddress(replaceValue(
        validConfigText(),
        "COMM_SERVER_ADDRESS",
        "300.1.1.1"));
    EXPECT_THROW(
        loadConfig(invalidAddress.path()),
        std::invalid_argument);

    std::string duplicatePeer = replaceValue(
        validConfigText(),
        "COMM_SCENARIO_SIMULATOR_ADDRESS",
        "127.0.0.2");
    duplicatePeer = replaceValue(
        duplicatePeer,
        "COMM_SCENARIO_SIMULATOR_PORT",
        "43201");
    const TemporaryEnvFile duplicatePeerFile(duplicatePeer);
    EXPECT_THROW(
        loadConfig(duplicatePeerFile.path()),
        std::invalid_argument);
}

TEST(AppConfigTest, MalformedDuplicateAndUnknownEntriesThrow)
{
    const TemporaryEnvFile malformed(validConfigText() + "MALFORMED\n");
    EXPECT_THROW(
        loadConfig(malformed.path()),
        std::runtime_error);

    const TemporaryEnvFile emptyKey(validConfigText() + "=value\n");
    EXPECT_THROW(
        loadConfig(emptyKey.path()),
        std::runtime_error);

    const TemporaryEnvFile duplicate(
        validConfigText() +
        "BEAM_MAX_ABS_PITCH_DEG=70\n");
    EXPECT_THROW(
        loadConfig(duplicate.path()),
        std::runtime_error);

    const TemporaryEnvFile unknown(validConfigText() + "UNKNOWN_KEY=1\n");
    EXPECT_THROW(
        loadConfig(unknown.path()),
        std::runtime_error);
}

} // namespace
