#pragma once

#include <protocol/ST_MsgHeader.hpp>

namespace comm::protocol
{

#pragma pack(push, 1)
struct ST_OperatorMsg
{
    ST_MsgHeader msgHeader;
    std::uint8_t oper_status;
    std::uint8_t padding[3];
};
#pragma pack(pop)

static_assert(sizeof(ST_OperatorMsg) == 24, "ST_OperatorMsg must be 24 bytes");

} // namespace comm::protocol
