#include <network/message_decoder.hpp>

#include <protocol/ST_OperatorMsg.hpp>
#include <protocol/ST_PlotMsg.hpp>
#include <protocol/ST_RadarAttMsg.hpp>

#include <cmath>
#include <cstddef>
#include <cstring>
#include <type_traits>
#include <utility>
#include <vector>

namespace comm
{
namespace
{

constexpr std::size_t PLOT_DATA_OFFSET =
    offsetof(protocol::ST_PlotMsg, plot);

constexpr std::size_t MAXIMUM_PLOT_COUNT =
    std::extent<decltype(protocol::ST_PlotMsg::plot)>::value;

template <typename Value>
Value readValue(
    const std::vector<uint8_t>& bytes,
    std::size_t offset = 0
)
{
    Value value{};
    std::memcpy(
        &value,
        bytes.data() + offset,
        sizeof(value));
    return value;
}

DecodeResult decoded(IncomingDto message)
{
    DecodeResult result;
    result.message.emplace(std::move(message));
    return result;
}

DecodeResult failed(DecodingError error)
{
    DecodeResult result;
    result.error = error;
    return result;
}

bool isFinite(const protocol::ST_RadarAttMsg& message)
{
    return std::isfinite(message.radar_lat_deg) &&
        std::isfinite(message.radar_lon_deg) &&
        std::isfinite(message.radar_alt_km) &&
        std::isfinite(message.roll_deg) &&
        std::isfinite(message.pitch_deg) &&
        std::isfinite(message.yaw_deg);
}

bool isFinite(const protocol::ST_Plot& plot)
{
    return std::isfinite(plot.slantR_km) &&
        std::isfinite(plot.azi_deg) &&
        std::isfinite(plot.elv_deg) &&
        std::isfinite(plot.doppler_mps) &&
        std::isfinite(plot.pwr_DB);
}

std::optional<beam::BeamCommand::BeamType> decodeBeamType(
    uint8_t beamType
)
{
    switch (static_cast<beam::BeamCommand::BeamType>(beamType))
    {
    case beam::BeamCommand::BeamType::SEARCH:
    case beam::BeamCommand::BeamType::CONFIRMATION:
    case beam::BeamCommand::BeamType::TRACKING:
        return static_cast<beam::BeamCommand::BeamType>(beamType);
    }

    return std::nullopt;
}

target::DetectionEvent::Plot decodePlot(
    const protocol::ST_Plot& plot
)
{
    return target::DetectionEvent::Plot::Builder()
        .slantRange(math::Distance::fromKilometers(plot.slantR_km))
        .azimuth(math::Angle::fromDegrees(plot.azi_deg))
        .elevation(math::Angle::fromDegrees(plot.elv_deg))
        .dopplerVelocity(
            math::Velocity::fromMetersPerSecond(plot.doppler_mps))
        .power_db(plot.pwr_DB)
        .build();
}

} // namespace

DecodeResult MessageDecoder::decode(
    const MessageFrame& frame
) const
{
    if (frame.bytes.size() != frame.header.block_size)
    {
        return failed(DecodingError::INVALID_MESSAGE_SIZE);
    }

    /*현재 version 사용 x
    if (frame.header.version != protocol::MESSAGE_VERSION)
    {
        return failed(DecodingError::UNSUPPORTED_VERSION);
    }
    */

    switch (static_cast<protocol::MessageId>(frame.header.message_Id))
    {
    case protocol::MessageId::OperatorControl:
        return decodeOperatorControl(frame);

    case protocol::MessageId::RadarAttitudeFromSimulator:
        return decodeRadarAttitude(frame);

    case protocol::MessageId::PlotInformation:
        return decodePlotInformation(frame);

    case protocol::MessageId::TrackInformation:
    case protocol::MessageId::RadarAttitudeToConsole:
    case protocol::MessageId::TrackDeletion:
    case protocol::MessageId::BeamTransmit:
        return failed(DecodingError::UNSUPPORTED_MESSAGE_ID);
    }

    return failed(DecodingError::UNSUPPORTED_MESSAGE_ID);
}

DecodeResult MessageDecoder::decodeOperatorControl(
    const MessageFrame& frame
) const
{
    if (frame.bytes.size() != sizeof(protocol::ST_OperatorMsg))
    {
        return failed(DecodingError::INVALID_MESSAGE_SIZE);
    }

    const protocol::ST_OperatorMsg message =
        readValue<protocol::ST_OperatorMsg>(frame.bytes);

    beam::OperationState operationState;
    switch (message.oper_status)
    {
    case 0:
        operationState = beam::OperationState::OFF;
        break;

    case 1:
        operationState = beam::OperationState::ON;
        break;

    default:
        return failed(DecodingError::INVALID_FIELD_VALUE);
    }

    return decoded(
        beam::SetOperationStateCommand::Builder()
            .powerState(operationState)
            .build());
}

DecodeResult MessageDecoder::decodeRadarAttitude(
    const MessageFrame& frame
) const
{
    if (frame.bytes.size() != sizeof(protocol::ST_RadarAttMsg))
    {
        return failed(DecodingError::INVALID_MESSAGE_SIZE);
    }

    const protocol::ST_RadarAttMsg message =
        readValue<protocol::ST_RadarAttMsg>(frame.bytes);

    if (!isFinite(message))
    {
        return failed(DecodingError::INVALID_FIELD_VALUE);
    }

    return decoded(
        beam::SetAttitudeCommand::Builder()
            .latitude(math::Angle::fromDegrees(message.radar_lat_deg))
            .longitude(math::Angle::fromDegrees(message.radar_lon_deg))
            .altitude(math::Distance::fromKilometers(message.radar_alt_km))
            .roll(math::Angle::fromDegrees(message.roll_deg))
            .pitch(math::Angle::fromDegrees(message.pitch_deg))
            .yaw(math::Angle::fromDegrees(message.yaw_deg))
            .build());
}

DecodeResult MessageDecoder::decodePlotInformation(
    const MessageFrame& frame
) const
{
    if (frame.bytes.size() < PLOT_DATA_OFFSET)
    {
        return failed(DecodingError::INVALID_MESSAGE_SIZE);
    }

    const protocol::ST_BeamTx beamCommand =
        readValue<protocol::ST_BeamTx>(
            frame.bytes,
            offsetof(protocol::ST_PlotMsg, beam_cmd));

    const protocol::ST_PlotHeader plotHeader =
        readValue<protocol::ST_PlotHeader>(
            frame.bytes,
            offsetof(protocol::ST_PlotMsg, plotHd));

    const std::size_t plotCount = plotHeader.number;
    if (plotCount > MAXIMUM_PLOT_COUNT)
    {
        return failed(DecodingError::INVALID_ITEM_COUNT);
    }

    const std::size_t expectedSize =
        PLOT_DATA_OFFSET + plotCount * sizeof(protocol::ST_Plot);
    if (frame.bytes.size() != expectedSize)
    {
        return failed(DecodingError::INVALID_MESSAGE_SIZE);
    }

    const std::optional<beam::BeamCommand::BeamType> beamType =
        decodeBeamType(beamCommand.beamType);
    if (!beamType.has_value())
    {
        return failed(DecodingError::INVALID_FIELD_VALUE);
    }

    std::vector<target::DetectionEvent::Plot> plots;
    plots.reserve(plotCount);

    for (std::size_t index = 0; index < plotCount; ++index)
    {
        const protocol::ST_Plot plot =
            readValue<protocol::ST_Plot>(
                frame.bytes,
                PLOT_DATA_OFFSET + index * sizeof(protocol::ST_Plot));

        if (!isFinite(plot))
        {
            return failed(DecodingError::INVALID_FIELD_VALUE);
        }

        plots.push_back(decodePlot(plot));
    }

    return decoded(
        target::DetectionEvent::Builder()
            .beamType(*beamType)
            .requestId(beamCommand.beamID)
            .transmitTime(
                std::chrono::milliseconds(beamCommand.timestamp))
            .plots(std::move(plots))
            .build());
}

} // namespace comm
