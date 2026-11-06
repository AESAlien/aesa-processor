#pragma once

#include <network/message_framer/message_frame.hpp>

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
