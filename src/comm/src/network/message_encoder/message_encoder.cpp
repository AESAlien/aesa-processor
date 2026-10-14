#include <network/message_encoder/message_encoder.hpp>

#include <protocol/ST_BeamTxMsg.hpp>
#include <protocol/ST_TrkMsg.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

namespace comm
{
namespace
{

constexpr std::size_t MAXIMUM_TRACK_COUNT =
    std::extent<decltype(protocol::ST_TrkMsg::trk)>::value;

constexpr std::size_t TRACK_DATA_OFFSET =
    sizeof(protocol::ST_MsgHeader) + sizeof(protocol::ST_TrkHeader);

EncodeResult encoded(std::vector<uint8_t> bytes)
{
    EncodeResult result;
    result.bytes.emplace(std::move(bytes));
    return result;
}

EncodeResult failed(EncodingError error)
{
    EncodeResult result;
    result.error = error;
    return result;
}

// Creates the common header shared by protocol message structures.
protocol::ST_MsgHeader makeHeader(
    protocol::MessageId messageId,
    std::size_t blockSize
)
{
    protocol::ST_MsgHeader header{};
    header.message_Id = static_cast<uint16_t>(messageId);
    header.block_size = static_cast<uint32_t>(blockSize);
    return header;
}

// Converts a DTO value to the protocol float representation.
std::optional<float> convertFloat(double value)
{
    if (std::isnan(value))
    {
        return static_cast<float>(value);
    }

    constexpr double MAXIMUM_FLOAT =
        static_cast<double>(std::numeric_limits<float>::max());

    if (!std::isfinite(value) ||
        value < -MAXIMUM_FLOAT ||
        value > MAXIMUM_FLOAT)
    {
        return std::nullopt;
    }

    return static_cast<float>(value);
}

// Converts a DTO timestamp to the protocol uint32 representation.
std::optional<uint32_t> convertTimestamp(
    std::chrono::milliseconds timestamp
)
{
    const std::chrono::milliseconds::rep count = timestamp.count();
    if (count < 0 ||
        static_cast<uint64_t>(count) >
            std::numeric_limits<uint32_t>::max())
    {
        return std::nullopt;
    }

    return static_cast<uint32_t>(count);
}

std::optional<protocol::ST_BeamTx> encodeBeam(
    const beam::BeamCommand& command
)
{
    const std::optional<uint32_t> timestamp =
        convertTimestamp(command.transmitTime);
    const std::optional<float> azimuth =
        convertFloat(command.azimuth_ant.deg());
    const std::optional<float> elevation =
        convertFloat(command.elevation_ant.deg());
    const std::optional<float> azimuthBeamWidth =
        convertFloat(command.azimuthBeamWidth.deg());
    const std::optional<float> elevationBeamWidth =
        convertFloat(command.elevationBeamWidth.deg());

    if (!timestamp ||
        !azimuth ||
        !elevation ||
        !azimuthBeamWidth ||
        !elevationBeamWidth)
    {
        return std::nullopt;
    }

    protocol::ST_BeamTx encodedBeam{};
    encodedBeam.beamType = static_cast<uint8_t>(command.beamType);
    encodedBeam.beamID = command.beamId;
    encodedBeam.commandCount = command.commandCount;
    encodedBeam.timestamp = *timestamp;
    encodedBeam.beam_az_deg = *azimuth;
    encodedBeam.beam_el_deg = *elevation;
    encodedBeam.az_width_deg = *azimuthBeamWidth;
    encodedBeam.el_width_deg = *elevationBeamWidth;

    return encodedBeam;
}

std::optional<protocol::ST_Trk> encodeTrack(
    const target::TrackEvent& event
)
{
    const std::optional<float> slantRange =
        convertFloat(event.slantRange.km());
    const std::optional<float> groundRange =
        convertFloat(event.groundRange.km());
    const std::optional<float> azimuth =
        convertFloat(event.azimuth.deg());
    const std::optional<float> elevation =
        convertFloat(event.elevation.deg());
    const std::optional<float> dopplerVelocity =
        convertFloat(event.dopplerVelocity.mps());
    const std::optional<float> latitude =
        convertFloat(event.latitude.deg());
    const std::optional<float> longitude =
        convertFloat(event.longitude.deg());
    const std::optional<float> altitude =
        convertFloat(event.altitude.km());
    const std::optional<float> velocity =
        convertFloat(event.velocity.mps());
    const std::optional<float> heading =
        convertFloat(event.heading.deg());
    const std::optional<float> flightPathAngle =
        convertFloat(event.flightPathAngle.deg());
    const std::optional<uint32_t> timestamp =
        convertTimestamp(event.timestamp);

    if (!slantRange ||
        !groundRange ||
        !azimuth ||
        !elevation ||
        !dopplerVelocity ||
        !latitude ||
        !longitude ||
        !altitude ||
        !velocity ||
        !heading ||
        !flightPathAngle ||
        !timestamp)
    {
        return std::nullopt;
    }

    protocol::ST_Trk encodedTrack{};
    encodedTrack.ID = event.id;
    encodedTrack.type = static_cast<uint8_t>(event.state);
    encodedTrack.slantR_km = *slantRange;
    encodedTrack.groundR_km = *groundRange;
    encodedTrack.azi_deg = *azimuth;
    encodedTrack.elv_deg = *elevation;
    encodedTrack.doppler_mps = *dopplerVelocity;
    encodedTrack.lat_deg = *latitude;
    encodedTrack.lon_deg = *longitude;
    encodedTrack.alt_km = *altitude;
    encodedTrack.velocity_mps = *velocity;
    encodedTrack.heading_deg = *heading;
    encodedTrack.fpa_deg = *flightPathAngle;
    encodedTrack.timeStamp = *timestamp;

    return encodedTrack;
}

// Serializes a protocol message for TCP transmission.
template <typename Message>
std::vector<uint8_t> toBytes(const Message& message)
{
    static_assert(
        std::is_trivially_copyable<Message>::value,
        "Protocol messages must be trivially copyable");

    std::vector<uint8_t> bytes(sizeof(message));
    std::memcpy(bytes.data(), &message, sizeof(message));
    return bytes;
}

} // namespace

EncodeResult encodeBeamCommand(
    const beam::BeamCommand& command
)
{
    const std::optional<protocol::ST_BeamTx> encodedBeam =
        encodeBeam(command);
    if (!encodedBeam)
    {
        return failed(EncodingError::TYPE_CONVERSION_ERROR);
    }

    protocol::ST_BeamTxMsg message{};
    message.msgHeader = makeHeader(
        protocol::MessageId::BeamTransmit,
        sizeof(message));
    message.beam_cmd = *encodedBeam;

    return encoded(toBytes(message));
}

EncodeResult encodeTrackEvents(
    const std::vector<target::TrackEvent>& trackEvents
)
{
    if (trackEvents.size() > MAXIMUM_TRACK_COUNT)
    {
        return failed(EncodingError::TOO_MANY_TRACKS);
    }

    std::vector<protocol::ST_Trk> encodedTracks;
    encodedTracks.reserve(trackEvents.size());
    for (const target::TrackEvent& trackEvent : trackEvents)
    {
        std::optional<protocol::ST_Trk> encodedTrack =
            encodeTrack(trackEvent);
        if (!encodedTrack)
        {
            return failed(EncodingError::TYPE_CONVERSION_ERROR);
        }

        encodedTracks.push_back(std::move(*encodedTrack));
    }

    const std::size_t blockSize =
        TRACK_DATA_OFFSET + encodedTracks.size() * sizeof(protocol::ST_Trk);
    const protocol::ST_MsgHeader header = makeHeader(
        protocol::MessageId::TrackInformation,
        blockSize);

    protocol::ST_TrkHeader trackHeader{};
    trackHeader.number = static_cast<uint16_t>(encodedTracks.size());

    std::vector<uint8_t> bytes(blockSize);
    std::size_t offset = 0;

    std::memcpy(bytes.data() + offset, &header, sizeof(header));
    offset += sizeof(header);

    std::memcpy(
        bytes.data() + offset,
        &trackHeader,
        sizeof(trackHeader));
    offset += sizeof(trackHeader);

    if (!encodedTracks.empty())
    {
        std::memcpy(
            bytes.data() + offset,
            encodedTracks.data(),
            encodedTracks.size() * sizeof(protocol::ST_Trk));
    }

    return encoded(std::move(bytes));
}

} // namespace comm
