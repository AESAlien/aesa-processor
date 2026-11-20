#include "config_assembler.hpp"

#include <math/quantity/angle.hpp>
#include <math/bounded/circular_angle.hpp>
#include <math/bounded/speed.hpp>
#include <math/bounded/tilt_angle.hpp>
#include <math/quantity/length.hpp>
#include <math/quantity/linear_acceleration.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace app
{

ConfigAssembler::ConfigAssembler(ConfigValueReader values)
  : _reader(std::move(values))
{
}

Config ConfigAssembler::assemble()
{
    pose::PoseConfig poseConfig = createPoseConfig();
    beam::BeamConfig beamConfig = createBeamConfig();
    target::TargetConfig targetConfig = createTargetConfig();
    comm::CommConfig commConfig = createCommConfig();
    _reader.ensureAllConsumed();

    return Config(
        std::move(poseConfig),
        std::move(beamConfig),
        std::move(targetConfig),
        std::move(commConfig)
    );
}

pose::PoseConfig ConfigAssembler::createPoseConfig()
{
    try
    {
        const double maxAbsPitch_deg = _reader.requireDouble("POSE_MAX_ABS_PITCH_DEG");
        const auto maxAbsPitch = math::Angle::fromDegrees(maxAbsPitch_deg);
        const auto attitudeConfig = pose::RadarAttitudeFactoryConfig(maxAbsPitch);
        pose::AntEnuTransformConfig antEnuTransformConfig(
            _reader.requireSquareMatrix("POSE_MOUNT_ANT_TO_BODY_ROTATION", 3)
        );
        return pose::PoseConfig(attitudeConfig, std::move(antEnuTransformConfig));
    }
    catch (const std::invalid_argument& error)
    {
        throw std::runtime_error(
            "Invalid Pose configuration: " + std::string(error.what())
        );
    }
}

beam::BeamConfig ConfigAssembler::createBeamConfig()
{
    try
    {
        const std::size_t searchAzimuthCount =
            _reader.requireUnsigned<std::size_t>("BEAM_SEARCH_AZIMUTH_COUNT");
        const std::size_t searchElevationCount =
            _reader.requireUnsigned<std::size_t>("BEAM_SEARCH_ELEVATION_COUNT");
        const double searchAzimuthStart_deg =
            _reader.requireDouble("BEAM_SEARCH_AZIMUTH_START_DEG");
        const double searchAzimuthStep_deg =
            _reader.requireDouble("BEAM_SEARCH_AZIMUTH_STEP_DEG");
        const double searchElevationStart_deg =
            _reader.requireDouble("BEAM_SEARCH_ELEVATION_START_DEG");
        const double searchElevationStep_deg =
            _reader.requireDouble("BEAM_SEARCH_ELEVATION_STEP_DEG");
        const double searchWidth_deg =
            _reader.requireDouble("BEAM_SEARCH_WIDTH_DEG");
        const double confirmationWidth_deg =
            _reader.requireDouble("BEAM_CONFIRMATION_WIDTH_DEG");
        const double trackingWidth_deg =
            _reader.requireDouble("BEAM_TRACKING_WIDTH_DEG");
        const double requestCoverageAzimuthMin_deg =
            _reader.requireDouble("BEAM_REQUEST_COVERAGE_AZIMUTH_MIN_DEG");
        const double requestCoverageAzimuthMax_deg =
            _reader.requireDouble("BEAM_REQUEST_COVERAGE_AZIMUTH_MAX_DEG");
        const double requestCoverageElevationMin_deg =
            _reader.requireDouble("BEAM_REQUEST_COVERAGE_ELEVATION_MIN_DEG");
        const double requestCoverageElevationMax_deg =
            _reader.requireDouble("BEAM_REQUEST_COVERAGE_ELEVATION_MAX_DEG");

        const auto searchWidth = math::Angle::fromDegrees(searchWidth_deg);
        const auto confirmationWidth = math::Angle::fromDegrees(confirmationWidth_deg);
        const auto trackingWidth = math::Angle::fromDegrees(trackingWidth_deg);
        const auto searchBeamWidth = beam::BeamWidth(searchWidth, searchWidth);
        const auto confirmationBeamWidth = beam::BeamWidth(confirmationWidth, confirmationWidth);
        const auto trackingBeamWidth = beam::BeamWidth(trackingWidth, trackingWidth);

        auto searchGridConfig = beam::SearchGridConfig(
            searchAzimuthCount,
            searchElevationCount,
            math::CircularAngle::fromDegrees(searchAzimuthStart_deg),
            math::Angle::fromDegrees(searchAzimuthStep_deg),
            math::TiltAngle::fromDegrees(searchElevationStart_deg),
            math::Angle::fromDegrees(searchElevationStep_deg),
            searchBeamWidth
        );
        auto requestBeamConfig = beam::RequestBeamConfig(
            confirmationBeamWidth,
            trackingBeamWidth
        );

        const auto azimuthRange = math::CircularAngleRange(
            math::Angle::fromDegrees(requestCoverageAzimuthMin_deg),
            math::Angle::fromDegrees(requestCoverageAzimuthMax_deg)
        );
        const auto elevationRange = math::AngleRange(
            math::Angle::fromDegrees(requestCoverageElevationMin_deg),
            math::Angle::fromDegrees(requestCoverageElevationMax_deg)
        );
        auto requestCoverageConfig = beam::RequestCoverageConfig(azimuthRange, elevationRange);

        return beam::BeamConfig(
            std::move(searchGridConfig),
            std::move(requestBeamConfig),
            std::move(requestCoverageConfig)
        );
    }
    catch (const std::invalid_argument& error)
    {
        throw std::runtime_error(
            "Invalid Beam configuration: " + std::string(error.what())
        );
    }
}

target::TargetConfig ConfigAssembler::createTargetConfig()
{
    try
    {
        const double maxAssociationSpeed_mps =
            _reader.requireDouble("TARGET_MAX_ASSOCIATION_SPEED_MPS");
        const double maxInitialDopplerDifference_mps =
            _reader.requireDouble(
                "TARGET_MAX_INITIAL_DOPPLER_DIFFERENCE_MPS"
            );
        const double measurementStandardDeviation_m =
            _reader.requireDouble(
                "TARGET_MEASUREMENT_STANDARD_DEVIATION_M"
            );
        const double positionNdsThreshold =
            _reader.requireDouble("TARGET_POSITION_NDS_THRESHOLD");
        const std::chrono::milliseconds confirmationDelay =
            _reader.requireDuration<std::chrono::milliseconds>(
                "TARGET_CONFIRMATION_DELAY_MS"
            );
        const std::chrono::milliseconds trackingInterval =
            _reader.requireDuration<std::chrono::milliseconds>(
                "TARGET_TRACKING_INTERVAL_MS"
            );
        const std::uint32_t maxConsecutiveTrackingMisses =
            _reader.requireUnsigned<std::uint32_t>(
                "TARGET_MAX_CONSECUTIVE_TRACKING_MISSES"
            );
        const double initialAccelerationStandardDeviation_mps2 =
            _reader.requireDouble(
                "TARGET_INITIAL_ACCELERATION_STANDARD_DEVIATION_MPS2"
            );
        const std::size_t maxReportedTrackCount =
            _reader.requireUnsigned<std::size_t>(
                "TARGET_MAX_REPORTED_TRACK_COUNT"
            );
        const double processAccelerationStandardDeviation_mps2 =
            _reader.requireDouble(
                "TARGET_PROCESS_ACCELERATION_STANDARD_DEVIATION_MPS2"
            );

        auto associationConfig = target::AssociationConfig(
            math::Speed::fromMetersPerSecond(maxAssociationSpeed_mps),
            math::Speed::fromMetersPerSecond(maxInitialDopplerDifference_mps),
            positionNdsThreshold
        );
        const auto measurementDeviation = math::Length::fromMeters(measurementStandardDeviation_m);
        auto measurementConfig = target::MeasurementConfig(measurementDeviation);
        const auto initialAccelerationDeviation = math::LinearAcceleration::fromMetersPerSecondSquared(
            initialAccelerationStandardDeviation_mps2
        );
        const auto processAccelerationDeviation = math::LinearAcceleration::fromMetersPerSecondSquared(
            processAccelerationStandardDeviation_mps2
        );
        auto trackingConfig = target::TrackingConfig(
            initialAccelerationDeviation,
            processAccelerationDeviation
        );
        auto trackManagementConfig = target::TrackManagementConfig(
            confirmationDelay,
            trackingInterval,
            maxConsecutiveTrackingMisses,
            maxReportedTrackCount
        );

        return target::TargetConfig(
            std::move(associationConfig),
            std::move(measurementConfig),
            std::move(trackingConfig),
            std::move(trackManagementConfig)
        );
    }
    catch (const std::invalid_argument& error)
    {
        throw std::runtime_error(
            "Invalid Target configuration: " + std::string(error.what())
        );
    }
}

comm::CommConfig ConfigAssembler::createCommConfig()
{
    const std::string& serverAddress =
        _reader.requireString("COMM_SERVER_ADDRESS");
    const std::uint16_t serverPort =
        _reader.requireUnsigned<std::uint16_t>("COMM_SERVER_PORT");
    const std::string& operatorConsoleAddress =
        _reader.requireString("COMM_OPERATOR_CONSOLE_ADDRESS");
    const std::uint16_t operatorConsolePort =
        _reader.requireUnsigned<std::uint16_t>(
            "COMM_OPERATOR_CONSOLE_PORT"
        );
    const std::string& scenarioSimulatorAddress =
        _reader.requireString("COMM_SCENARIO_SIMULATOR_ADDRESS");
    const std::uint16_t scenarioSimulatorPort =
        _reader.requireUnsigned<std::uint16_t>(
            "COMM_SCENARIO_SIMULATOR_PORT"
        );

    auto serverEndpoint = comm::CommEndpoint(serverAddress, serverPort);
    auto operatorConsoleEndpoint = comm::CommEndpoint(operatorConsoleAddress, operatorConsolePort);
    auto scenarioSimulatorEndpoint = comm::CommEndpoint(scenarioSimulatorAddress, scenarioSimulatorPort);

    return comm::CommConfig(
        std::move(serverEndpoint),
        std::move(operatorConsoleEndpoint),
        std::move(scenarioSimulatorEndpoint)
    );
}

} // namespace app
