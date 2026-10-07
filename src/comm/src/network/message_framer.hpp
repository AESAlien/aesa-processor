#pragma once

#include <protocol/ST_MsgHeader.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace comm
{

struct MessageFrame
{
    protocol::ST_MsgHeader header;
    std::vector<std::uint8_t> bytes;
};

enum class FramingError
{
    NONE,
    UNKNOWN_MESSAGE_ID,
    INVALID_FRAME_SIZE
};

struct FrameResult
{
    std::vector<MessageFrame> frames;
    FramingError error = FramingError::NONE;
};

class MessageFramer
{
public:
    static constexpr std::size_t DEFAULT_MAXIMUM_FRAME_SIZE =
        64U * 1024U;

    explicit MessageFramer(
        std::size_t maximumFrameSize = DEFAULT_MAXIMUM_FRAME_SIZE
    );

    FrameResult append(const std::vector<std::uint8_t>& bytes);

    void reset();

private:
    protocol::ST_MsgHeader peekHeader() const;

    FramingError validateHeader(
        const protocol::ST_MsgHeader& header
    ) const;

    bool isKnownMessageId(std::uint16_t messageId) const;

    std::size_t getFrameSize(
        const protocol::ST_MsgHeader& header
    ) const;

    MessageFrame extractFrame(
        const protocol::ST_MsgHeader& header,
        std::size_t frameSize
    );

    std::size_t availableSize() const;
    void compactBuffer();

    std::vector<std::uint8_t> _buffer;
    std::size_t _readOffset = 0;
    std::size_t _maximumFrameSize;
};

} // namespace comm
