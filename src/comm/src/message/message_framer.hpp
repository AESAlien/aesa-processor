#pragma once

#include <message/dto/message_frame.hpp>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace comm
{

// 버퍼의 바이트로 유효한 프레임을 구성할 수 없을 때 던진다.
// 메시지에는 "invalid-frame-size"와 같은 오류 원인이 담긴다.
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

    // 완성된 프레임을 반환한다. FramingError를 던질 수 있다.
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
