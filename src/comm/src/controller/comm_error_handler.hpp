#pragma once

#include <network/message_decoder/message_decoder.hpp>
#include <network/message_encoder/message_encoder.hpp>
#include <network/message_framer/message_framer.hpp>
#include <network/network_manager/network_types.hpp>

namespace aesa_processor::comm
{
namespace controller_error
{

[[noreturn]] void throwServerOpenFailure(
    const OperationResult& result
);

[[noreturn]] void throwPollFailure(
    const PollResult& result
);

[[noreturn]] void throwReceiveFailure(
    NetworkOperation operation,
    int errorCode
);

[[noreturn]] void throwFramingFailure(
    NetworkPeer peer,
    FramingError error
);

[[noreturn]] void throwDecodingFailure(
    NetworkPeer peer,
    DecodingError error
);

[[noreturn]] void throwEncodingFailure(
    NetworkPeer peer,
    EncodingError error
);

[[noreturn]] void throwSendFailure(
    NetworkPeer peer,
    const SendResult& result
);

void reportRejectedConnection(
    const ConnectionRejectedEvent& event
) noexcept;

} // namespace controller_error
} // namespace aesa_processor::comm
