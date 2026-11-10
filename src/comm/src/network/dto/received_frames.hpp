#pragma once

#include <message/dto/message_frame.hpp>

#include <string>
#include <vector>

namespace comm
{

struct ReceivedFrames
{
    std::string peerName;
    std::vector<MessageFrame> frames;
};

} // namespace comm
