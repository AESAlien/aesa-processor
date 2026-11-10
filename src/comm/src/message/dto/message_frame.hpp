#pragma once

#include <protocol/ST_MsgHeader.hpp>

#include <cstdint>
#include <vector>

namespace comm
{

struct MessageFrame
{
    protocol::ST_MsgHeader header;
    std::vector<uint8_t> bytes;
};

} // namespace comm
