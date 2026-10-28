#include "config_assembler.hpp"

#include <math/quantity/angle.hpp>
#include <math/geometry/angular_width.hpp>
#include <math/geometry/azimuth.hpp>
#include <math/geometry/elevation.hpp>
#include <math/quantity/length.hpp>
#include <math/quantity/linear_acceleration.hpp>
#include <math/quantity/linear_velocity.hpp>
#include <math/tolerance.hpp>

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
    beam::BeamConfig beamConfig = createBeamConfig();
    target::TargetConfig targetConfig = createTargetConfig();
    comm::CommConfig commConfig = createCommConfig();
    _reader.ensureAllConsumed();

    return Config(
        std::move(beamConfig),
        std::move(targetConfig),
        std::move(commConfig));
}

beam::BeamConfig ConfigAssembler::createBeamConfig()
{
    try
    {
        const double MAX_ABS_PITCH_DEG =
            _reader.requireDouble("BEAM_MAX_ABS_PITCH_DEG");
        const math::SquareMatrix MOUNT_ANT_TO_BODY_ROTATION =
            _reader.requireSquareMatrix("BEAM_MOUNT_ANT_TO_BODY_ROTATION", 3);
        const double MOUNT_ORTHOGONAL_ABSOLUTE_TOLERANCE =
            _reader.requireDouble("MOUNT_ORTHOGONAL_ABSOLUTE_TOLERANCE");
        const double MOUNT_ORTHOGONAL_RELATIVE_TOLERANCE =
            _reader.requireDouble("MOUNT_ORTHOGONAL_RELATIVE_TOLERANCE");
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

        return beam::BeamConfig(
            beam::AttitudeConfig(
                math::Angle::fromDegrees(MAX_ABS_PITCH_DEG),
                MOUNT_ANT_TO_BODY_ROTATION,
                math::Tolerance{
                    MOUNT_ORTHOGONAL_ABSOLUTE_TOLERANCE,
                    MOUNT_ORTHOGONAL_RELATIVE_TOLERANCE}),
            beam::SearchGridConfig(
                SEARCH_AZIMUTH_COUNT,
                SEARCH_ELEVATION_COUNT,
                math::Azimuth::fromDegrees(SEARCH_AZIMUTH_START_DEG),
                math::Angle::fromDegrees(SEARCH_AZIMUTH_STEP_DEG),
                math::Elevation::fromDegrees(SEARCH_ELEVATION_START_DEG),
                math::Angle::fromDegrees(SEARCH_ELEVATION_STEP_DEG),
                beam::BeamWidth(
                    math::AngularWidth(math::Angle::fromDegrees(SEARCH_WIDTH_DEG)),
                    math::AngularWidth(math::Angle::fromDegrees(SEARCH_WIDTH_DEG)))),
            beam::RequestBeamConfig(
                beam::BeamWidth(
                    math::AngularWidth(math::Angle::fromDegrees(CONFIRMATION_WIDTH_DEG)),
                    math::AngularWidth(math::Angle::fromDegrees(CONFIRMATION_WIDTH_DEG))),
                beam::BeamWidth(
                    math::AngularWidth(math::Angle::fromDegrees(TRACKING_WIDTH_DEG)),
                    math::AngularWidth(math::Angle::fromDegrees(TRACKING_WIDTH_DEG)))));
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
            math::LinearVelocity::fromMetersPerSecond(MAX_ASSOCIATION_SPEED_MPS),
            math::LinearVelocity::fromMetersPerSecond(MAX_INITIAL_DOPPLER_DIFFERENCE_MPS),
            math::Length::fromMeters(MEASUREMENT_STANDARD_DEVIATION_M),
            POSITION_NDS_THRESHOLD,
            CONFIRMATION_DELAY,
            TRACKING_INTERVAL,
            MAX_CONSECUTIVE_TRACKING_MISSES,
            math::LinearAcceleration::fromMetersPerSecondSquared(
                INITIAL_ACCELERATION_STANDARD_DEVIATION_MPS2),
            MAX_REPORTED_TRACK_COUNT,
            math::LinearAcceleration::fromMetersPerSecondSquared(
                PROCESS_ACCELERATION_STANDARD_DEVIATION_MPS2));
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
