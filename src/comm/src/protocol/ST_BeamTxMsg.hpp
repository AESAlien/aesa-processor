#pragma once

#include <protocol/ST_MsgHeader.hpp>

#include <cstdint>

namespace comm::protocol
{

#pragma pack(push, 1)
struct ST_BeamTx
{
    std::uint8_t beamType;
    std::uint8_t padding[3];
    std::uint32_t timestamp;
    std::uint32_t beamID;
    std::uint32_t commandCount;
    float beam_az_deg;
    float beam_el_deg;
    float az_width_deg;
    float el_width_deg;
    std::uint8_t padding_data[28];
};

struct ST_BeamTxMsg
{
    ST_MsgHeader msgHeader;
    ST_BeamTx beam_cmd;
};
#pragma pack(pop)

static_assert(sizeof(ST_BeamTx) == 60, "ST_BeamTx must be 60 bytes");
static_assert(sizeof(ST_BeamTxMsg) == 80, "ST_BeamTxMsg must be 80 bytes");

} // namespace comm::protocol
