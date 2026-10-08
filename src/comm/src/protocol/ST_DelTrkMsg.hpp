#pragma once

#include <protocol/ST_TrkMsg.hpp>

#include <cstdint>

namespace comm::protocol
{

#pragma pack(push, 1)
struct ST_DelTrkMsg
{
    ST_MsgHeader msgHeader;
    ST_TrkHeader trkHd;
    uint16_t delTrkID[100];
};
#pragma pack(pop)

static_assert(sizeof(ST_DelTrkMsg) == 224, "ST_DelTrkMsg must be 224 bytes");

} // namespace comm::protocol
