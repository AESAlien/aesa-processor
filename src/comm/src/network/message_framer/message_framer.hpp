#pragma once

#include <protocol/ST_MsgHeader.hpp>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace comm
{

struct MessageFrame
{
    protocol::ST_MsgHeader header;
    std::vector<uint8_t> bytes;
};

// Thrown when the buffered bytes cannot form a valid frame.
// The message names the cause, such as "invalid-frame-size".
class FramingError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class MessageFramer
{
public:
    static constexpr std::size_t DEFAULT_MAXIMUM_FRAME_SIZE =
        64U * 1024U;

    explicit MessageFramer(
        std::size_t maximumFrameSize = DEFAULT_MAXIMUM_FRAME_SIZE
    );

    // Returns the completed frames. Throws FramingError.
    std::vector<MessageFrame> append(const std::vector<uint8_t>& bytes);

    void reset();

private:
    void validate() const;

    protocol::ST_MsgHeader peekHeader() const;

    void validateHeader(
        const protocol::ST_MsgHeader& header
    ) const;

    bool isKnownMessageId(uint16_t messageId) const;

    std::size_t getFrameSize(
        const protocol::ST_MsgHeader& header
    ) const;

    MessageFrame extractFrame(
        const protocol::ST_MsgHeader& header,
        std::size_t frameSize
    );

    std::size_t availableSize() const;
    void compactBuffer();

    std::vector<uint8_t> _buffer;
    std::size_t _readOffset = 0;
    std::size_t _maximumFrameSize;
};

} // namespace comm
