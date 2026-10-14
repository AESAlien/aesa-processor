#pragma once

#include <comm/dto/incoming_dto.hpp>
#include <network/message_framer.hpp>

#include <optional>

namespace comm
{

enum class DecodingError
{
    NONE,
    UNSUPPORTED_MESSAGE_ID,
    UNSUPPORTED_VERSION,
    INVALID_MESSAGE_SIZE,
    INVALID_ITEM_COUNT,
    INVALID_FIELD_VALUE
};

struct DecodeResult
{
    std::optional<IncomingDto> message;
    DecodingError error = DecodingError::NONE;
};

class MessageDecoder
{
public:
    DecodeResult decode(const MessageFrame& frame) const;

private:
    DecodeResult decodeOperatorControl(
        const MessageFrame& frame
    ) const;

    DecodeResult decodeRadarAttitude(
        const MessageFrame& frame
    ) const;

    DecodeResult decodePlotInformation(
        const MessageFrame& frame
    ) const;
};

} // namespace comm
