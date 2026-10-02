#pragma once

#include <comm/protocol/ST_MsgHeader.hpp>

#include <cstdint>

namespace comm::protocol
{

#pragma pack(push, 1)
struct ST_TrkHeader
{
    std::uint16_t number;
    std::uint8_t padding[2];
};

struct ST_Trk
{
    std::uint16_t ID;
    std::uint8_t type;
    std::uint8_t memFlag;
    float slantR_km;
    float groundR_km;
    float azi_deg;
    float elv_deg;
    float doppler_mps;
    float lat_deg;
    float lon_deg;
    float alt_km;
    float velocity_mps;
    float heading_deg;
    float fpa_deg;
    std::uint32_t timeStamp;
    std::uint32_t beamID;
    std::uint32_t commandCount;
    std::uint8_t padding[28];
};

struct ST_TrkMsg
{
    ST_MsgHeader msgHeader;
    ST_TrkHeader trkHd;
    ST_Trk trk[100];
};
#pragma pack(pop)

static_assert(sizeof(ST_TrkHeader) == 4, "ST_TrkHeader must be 4 bytes");
static_assert(sizeof(ST_Trk) == 88, "ST_Trk must be 88 bytes");
static_assert(sizeof(ST_TrkMsg) == 8824, "Maximum ST_TrkMsg must be 8824 bytes");

} // namespace comm::protocol
