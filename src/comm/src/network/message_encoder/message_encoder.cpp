#include <network/message_encoder/message_encoder.hpp>

#include <protocol/ST_BeamTxMsg.hpp>
#include <protocol/ST_TrkMsg.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace aesa_processor::comm
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
float convertFloat(double value)
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
        throw std::out_of_range(
            "Value cannot be represented as protocol float");
    }

    return static_cast<float>(value);
}

// Converts a DTO timestamp to the protocol uint32 representation.
uint32_t convertTimestamp(
    std::chrono::milliseconds timestamp
)
{
    const std::chrono::milliseconds::rep count = timestamp.count();
    if (count < 0 ||
        static_cast<uint64_t>(count) >
            std::numeric_limits<uint32_t>::max())
    {
        throw std::out_of_range(
            "Timestamp cannot be represented as protocol uint32");
    }

    return static_cast<uint32_t>(count);
}

protocol::ST_BeamTx encodeBeam(
    const aesa_processor::beam::TransmitBeamCommand& command
)
{
    protocol::ST_BeamTx encodedBeam{};
    encodedBeam.beamType =      static_cast<uint8_t>(command.beamType);
    encodedBeam.beamID =        command.beamId;
    encodedBeam.commandCount =  command.commandCount;
    encodedBeam.timestamp =     convertTimestamp(command.transmitTime);
    encodedBeam.beam_az_deg =   convertFloat(command.direction_ant.azimuth.deg());
    encodedBeam.beam_el_deg =   convertFloat(command.direction_ant.elevation.deg());
    encodedBeam.az_width_deg =  convertFloat(command.azimuthBeamWidth.deg());
    encodedBeam.el_width_deg =  convertFloat(command.elevationBeamWidth.deg());

    return encodedBeam;
}

protocol::ST_Trk encodeTrack(
    const aesa_processor::target::TrackEvent& event
)
{
    protocol::ST_Trk encodedTrack{};
    encodedTrack.ID =           event.id;
    encodedTrack.type =         static_cast<uint8_t>(event.state);
    encodedTrack.slantR_km =    convertFloat(event.slantRange.km());
    encodedTrack.groundR_km =   convertFloat(event.groundRange.km());
    encodedTrack.azi_deg =      convertFloat(event.azimuth.deg());
    encodedTrack.elv_deg =      convertFloat(event.elevation.deg());
    encodedTrack.doppler_mps =  convertFloat(event.dopplerVelocity.mps());
    encodedTrack.lat_deg =      convertFloat(event.latitude.deg());
    encodedTrack.lon_deg =      convertFloat(event.longitude.deg());
    encodedTrack.alt_km =       convertFloat(event.altitude.km());
    encodedTrack.velocity_mps = convertFloat(event.velocity.mps());
    encodedTrack.heading_deg =  convertFloat(event.heading.deg());
    encodedTrack.fpa_deg =      convertFloat(event.flightPathAngle.deg());
    encodedTrack.timeStamp =    convertTimestamp(event.timestamp);

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
    const aesa_processor::beam::TransmitBeamCommand& command
)
{
    try
    {
        const protocol::ST_BeamTx encodedBeam = encodeBeam(command);

        protocol::ST_BeamTxMsg message{};
        message.msgHeader = makeHeader(
            protocol::MessageId::BeamTransmit,
            sizeof(message));
        message.beam_cmd = encodedBeam;

        return encoded(toBytes(message));
    }
    catch (const std::out_of_range&)
    {
        return failed(EncodingError::TYPE_CONVERSION_ERROR);
    }
}

EncodeResult encodeTrackEvents(
    const std::vector<aesa_processor::target::TrackEvent>& trackEvents
)
{
    if (trackEvents.size() > MAXIMUM_TRACK_COUNT)
    {
        return failed(EncodingError::TOO_MANY_TRACKS);
    }

    std::vector<protocol::ST_Trk> encodedTracks;
    encodedTracks.reserve(trackEvents.size());
    try
    {
        for (const aesa_processor::target::TrackEvent& trackEvent : trackEvents)
        {
            encodedTracks.push_back(encodeTrack(trackEvent));
        }
    }
    catch (const std::out_of_range&)
    {
        return failed(EncodingError::TYPE_CONVERSION_ERROR);
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

} // namespace aesa_processor::comm
