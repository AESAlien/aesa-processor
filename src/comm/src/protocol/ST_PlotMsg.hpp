#pragma once

#include <protocol/ST_BeamTxMsg.hpp>
#include <protocol/ST_MsgHeader.hpp>

#include <cstdint>

namespace comm::protocol
{

#pragma pack(push, 1)
struct ST_PlotHeader
{
    uint16_t number;
    uint8_t padding[2];
};

// NOLINTBEGIN(readability-identifier-naming)
struct ST_Plot
{
    uint32_t ID;
    float slantR_km;
    float azi_deg;
    float elv_deg;
    float doppler_mps;
    float pwr_DB;
};
// NOLINTEND(readability-identifier-naming)

// NOLINTBEGIN(readability-identifier-naming)
struct ST_PlotMsg
{
    ST_MsgHeader msgHeader;
    ST_BeamTx beam_cmd;
    ST_PlotHeader plotHd;
    ST_Plot plot[100];
};
// NOLINTEND(readability-identifier-naming)
#pragma pack(pop)

static_assert(sizeof(ST_PlotHeader) == 4, "ST_PlotHeader must be 4 bytes");
static_assert(sizeof(ST_Plot) == 24, "ST_Plot must be 24 bytes");
static_assert(sizeof(ST_PlotMsg) == 2484, "ST_PlotMsg must be 2484 bytes");

} // namespace comm::protocol
