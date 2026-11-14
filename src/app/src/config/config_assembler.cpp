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
        std::move(commConfig));
}

pose::PoseConfig ConfigAssembler::createPoseConfig()
{
    try
    {
        const pose::RadarAttitudeFactoryConfig attitude(
            math::Angle::fromDegrees(_reader.requireDouble("POSE_MAX_ABS_PITCH_DEG"))
        );
        pose::AntEnuTransformConfig antEnuTransformConfig(
            _reader.requireSquareMatrix("POSE_MOUNT_ANT_TO_BODY_ROTATION", 3)
        );
        return pose::PoseConfig(attitude, std::move(antEnuTransformConfig));
    }
    catch (const std::invalid_argument& error)
    {
        throw std::runtime_error(
            "Invalid Pose configuration: " + std::string(error.what()));
    }
}

beam::BeamConfig ConfigAssembler::createBeamConfig()
{
    try
    {
        const std::size_t SEARCH_AZIMUTH_COUNT =
            _reader.requireUnsigned<std::size_t>("BEAM_SEARCH_AZIMUTH_COUNT");
        const std::size_t SEARCH_ELEVATION_COUNT =
            _reader.requireUnsigned<std::size_t>("BEAM_SEARCH_ELEVATION_COUNT");
        const double SEARCH_AZIMUTH_START_DEG =
            _reader.requireDouble("BEAM_SEARCH_AZIMUTH_START_DEG");
        const double SEARCH_AZIMUTH_STEP_DEG =
            _reader.requireDouble("BEAM_SEARCH_AZIMUTH_STEP_DEG");
        const double SEARCH_ELEVATION_START_DEG =
            _reader.requireDouble("BEAM_SEARCH_ELEVATION_START_DEG");
        const double SEARCH_ELEVATION_STEP_DEG =
            _reader.requireDouble("BEAM_SEARCH_ELEVATION_STEP_DEG");
        const double SEARCH_WIDTH_DEG =
            _reader.requireDouble("BEAM_SEARCH_WIDTH_DEG");
        const double CONFIRMATION_WIDTH_DEG =
            _reader.requireDouble("BEAM_CONFIRMATION_WIDTH_DEG");
        const double TRACKING_WIDTH_DEG =
            _reader.requireDouble("BEAM_TRACKING_WIDTH_DEG");
        const double REQUEST_COVERAGE_AZIMUTH_MIN_DEG =
            _reader.requireDouble("BEAM_REQUEST_COVERAGE_AZIMUTH_MIN_DEG");
        const double REQUEST_COVERAGE_AZIMUTH_MAX_DEG =
            _reader.requireDouble("BEAM_REQUEST_COVERAGE_AZIMUTH_MAX_DEG");
        const double REQUEST_COVERAGE_ELEVATION_MIN_DEG =
            _reader.requireDouble("BEAM_REQUEST_COVERAGE_ELEVATION_MIN_DEG");
        const double REQUEST_COVERAGE_ELEVATION_MAX_DEG =
            _reader.requireDouble("BEAM_REQUEST_COVERAGE_ELEVATION_MAX_DEG");

        return beam::BeamConfig(
            beam::SearchGridConfig(
                SEARCH_AZIMUTH_COUNT,
                SEARCH_ELEVATION_COUNT,
                math::CircularAngle::fromDegrees(SEARCH_AZIMUTH_START_DEG),
                math::Angle::fromDegrees(SEARCH_AZIMUTH_STEP_DEG),
                math::TiltAngle::fromDegrees(SEARCH_ELEVATION_START_DEG),
                math::Angle::fromDegrees(SEARCH_ELEVATION_STEP_DEG),
                beam::BeamWidth(
                    math::Angle::fromDegrees(SEARCH_WIDTH_DEG),
                    math::Angle::fromDegrees(SEARCH_WIDTH_DEG))),
            beam::RequestBeamConfig(
                beam::BeamWidth(
                    math::Angle::fromDegrees(CONFIRMATION_WIDTH_DEG),
                    math::Angle::fromDegrees(CONFIRMATION_WIDTH_DEG)),
                beam::BeamWidth(
                    math::Angle::fromDegrees(TRACKING_WIDTH_DEG),
                    math::Angle::fromDegrees(TRACKING_WIDTH_DEG))),
            beam::RequestCoverageConfig(
                math::CircularAngleRange(
                    math::Angle::fromDegrees(REQUEST_COVERAGE_AZIMUTH_MIN_DEG),
                    math::Angle::fromDegrees(REQUEST_COVERAGE_AZIMUTH_MAX_DEG)),
                math::AngleRange(
                    math::Angle::fromDegrees(REQUEST_COVERAGE_ELEVATION_MIN_DEG),
                    math::Angle::fromDegrees(REQUEST_COVERAGE_ELEVATION_MAX_DEG))));
    }
    catch (const std::invalid_argument& error)
    {
        throw std::runtime_error(
            "Invalid Beam configuration: " + std::string(error.what()));
    }
}

target::TargetConfig ConfigAssembler::createTargetConfig()
{
    try
    {
        const double MAX_ASSOCIATION_SPEED_MPS =
            _reader.requireDouble("TARGET_MAX_ASSOCIATION_SPEED_MPS");
        const double MAX_INITIAL_DOPPLER_DIFFERENCE_MPS =
            _reader.requireDouble(
                "TARGET_MAX_INITIAL_DOPPLER_DIFFERENCE_MPS");
        const double MEASUREMENT_STANDARD_DEVIATION_M =
            _reader.requireDouble(
                "TARGET_MEASUREMENT_STANDARD_DEVIATION_M");
        const double POSITION_NDS_THRESHOLD =
            _reader.requireDouble("TARGET_POSITION_NDS_THRESHOLD");
        const std::chrono::milliseconds CONFIRMATION_DELAY =
            _reader.requireDuration<std::chrono::milliseconds>(
                "TARGET_CONFIRMATION_DELAY_MS");
        const std::chrono::milliseconds TRACKING_INTERVAL =
            _reader.requireDuration<std::chrono::milliseconds>(
                "TARGET_TRACKING_INTERVAL_MS");
        const std::uint32_t MAX_CONSECUTIVE_TRACKING_MISSES =
            _reader.requireUnsigned<std::uint32_t>(
                "TARGET_MAX_CONSECUTIVE_TRACKING_MISSES");
        const double INITIAL_ACCELERATION_STANDARD_DEVIATION_MPS2 =
            _reader.requireDouble(
                "TARGET_INITIAL_ACCELERATION_STANDARD_DEVIATION_MPS2");
        const std::size_t MAX_REPORTED_TRACK_COUNT =
            _reader.requireUnsigned<std::size_t>(
                "TARGET_MAX_REPORTED_TRACK_COUNT");
        const double PROCESS_ACCELERATION_STANDARD_DEVIATION_MPS2 =
            _reader.requireDouble(
                "TARGET_PROCESS_ACCELERATION_STANDARD_DEVIATION_MPS2");

        return target::TargetConfig(
            target::AssociationConfig(
                math::Speed::fromMetersPerSecond(MAX_ASSOCIATION_SPEED_MPS),
                math::Speed::fromMetersPerSecond(MAX_INITIAL_DOPPLER_DIFFERENCE_MPS),
                POSITION_NDS_THRESHOLD),
            target::MeasurementConfig(
                math::Length::fromMeters(MEASUREMENT_STANDARD_DEVIATION_M)),
            target::TrackingConfig(
                math::LinearAcceleration::fromMetersPerSecondSquared(
                    INITIAL_ACCELERATION_STANDARD_DEVIATION_MPS2),
                math::LinearAcceleration::fromMetersPerSecondSquared(
                    PROCESS_ACCELERATION_STANDARD_DEVIATION_MPS2)),
            target::TrackManagementConfig(
                CONFIRMATION_DELAY,
                TRACKING_INTERVAL,
                MAX_CONSECUTIVE_TRACKING_MISSES,
                MAX_REPORTED_TRACK_COUNT));
    }
    catch (const std::invalid_argument& error)
    {
        throw std::runtime_error(
            "Invalid Target configuration: " + std::string(error.what()));
    }
}

comm::CommConfig ConfigAssembler::createCommConfig()
{
    const std::string& SERVER_ADDRESS =
        _reader.requireString("COMM_SERVER_ADDRESS");
    const std::uint16_t SERVER_PORT =
        _reader.requireUnsigned<std::uint16_t>("COMM_SERVER_PORT");
    const std::string& OPERATOR_CONSOLE_ADDRESS =
        _reader.requireString("COMM_OPERATOR_CONSOLE_ADDRESS");
    const std::uint16_t OPERATOR_CONSOLE_PORT =
        _reader.requireUnsigned<std::uint16_t>(
            "COMM_OPERATOR_CONSOLE_PORT");
    const std::string& SCENARIO_SIMULATOR_ADDRESS =
        _reader.requireString("COMM_SCENARIO_SIMULATOR_ADDRESS");
    const std::uint16_t SCENARIO_SIMULATOR_PORT =
        _reader.requireUnsigned<std::uint16_t>(
            "COMM_SCENARIO_SIMULATOR_PORT");

    return comm::CommConfig(
        comm::CommEndpoint(SERVER_ADDRESS, SERVER_PORT),
        comm::CommEndpoint(
            OPERATOR_CONSOLE_ADDRESS, OPERATOR_CONSOLE_PORT),
        comm::CommEndpoint(
            SCENARIO_SIMULATOR_ADDRESS, SCENARIO_SIMULATOR_PORT));
}

} // namespace app
