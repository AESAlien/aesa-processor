#pragma once

#include <network/message_decoder/message_decoder.hpp>
#include <network/message_encoder/message_encoder.hpp>
#include <network/network_manager/network_types.hpp>

namespace comm
{
namespace controller_error
{

[[noreturn]] void throwDecodingFailure(
    NetworkPeer peer,
    DecodingError error
);

[[noreturn]] void throwEncodingFailure(
    NetworkPeer peer,
    EncodingError error
);

} // namespace controller_error
} // namespace comm
