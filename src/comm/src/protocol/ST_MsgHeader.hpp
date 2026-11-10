#pragma once

#include <cstdint>

namespace comm::protocol
{

constexpr uint8_t MESSAGE_VERSION = 1;

enum class MessageId : uint16_t
{
    OPERATOR_CONTROL = 0xCA01,
    TRACK_INFORMATION = 0xAC01,
    RADAR_ATTITUDE_TO_CONSOLE = 0xAC02,
    TRACK_DELETION = 0xAC03,
    BEAM_TRANSMIT = 0xAB01,
    PLOT_INFORMATION = 0xBA01,
    RADAR_ATTITUDE_FROM_SIMULATOR = 0xBA02,
};

#pragma pack(push, 1)
struct ST_MsgHeader
{
    uint16_t message_Id;
    uint8_t version;
    uint8_t padding1;
    uint32_t block_size;
    uint32_t timeSec;
    uint32_t timeNsec;
    uint8_t source_id;
    uint8_t dest_id;
    uint16_t padding2;
};
#pragma pack(pop)

static_assert(sizeof(ST_MsgHeader) == 20, "ST_MsgHeader must be 20 bytes");

} // namespace comm::protocol
