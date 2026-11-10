#pragma once

#include <message/message_decoder.hpp>
#include <message/message_encoder.hpp>
#include <string_view>

namespace comm
{
namespace controller_error
{

[[noreturn]] void throwDecodingFailure(
    std::string_view peerName,
    DecodingError error
);

[[noreturn]] void throwEncodingFailure(
    std::string_view peerName,
    EncodingError error
);

} // namespace controller_error
} // namespace comm
