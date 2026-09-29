#pragma once

#include <cstdint>

namespace comm::protocol
{

enum class MessageId : std::uint16_t
{
    OperatorControl = 0xCA01,
    TrackInformation = 0xAC01,
    RadarAttitudeToConsole = 0xAC02,
    TrackDeletion = 0xAC03,
    BeamTransmit = 0xAB01,
    PlotInformation = 0xBA01,
    RadarAttitudeFromSimulator = 0xBA02,
};

#pragma pack(push, 1)
struct ST_MsgHeader
{
    std::uint16_t message_Id;
    std::uint8_t version;
    std::uint8_t padding1;
    std::uint32_t block_size;
    std::uint32_t timeSec;
    std::uint32_t timeNsec;
    std::uint8_t source_id;
    std::uint8_t dest_id;
    std::uint16_t padding2;
};
#pragma pack(pop)

static_assert(sizeof(ST_MsgHeader) == 20, "ST_MsgHeader must be 20 bytes");

} // namespace comm::protocol
