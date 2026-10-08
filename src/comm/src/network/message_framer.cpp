#include <network/message_framer.hpp>

#include <cstring>
#include <stdexcept>
#include <utility>

namespace comm
{

MessageFramer::MessageFramer(std::size_t maximumFrameSize) :
    _maximumFrameSize(maximumFrameSize)
{
    if (_maximumFrameSize < sizeof(protocol::ST_MsgHeader))
    {
        throw std::invalid_argument(
            "Maximum frame size must be at least the header size");
    }

    _buffer.reserve(_maximumFrameSize);
}

FrameResult MessageFramer::append(
    const std::vector<uint8_t>& bytes
)
{
    FrameResult result;

    _buffer.insert(
        _buffer.end(),
        bytes.begin(),
        bytes.end()
    );

    while (availableSize() >= sizeof(protocol::ST_MsgHeader))
    {
        const protocol::ST_MsgHeader header = peekHeader();

        const FramingError validationError = validateHeader(header);
        if (validationError != FramingError::NONE)
        {
            result.error = validationError;
            return result;
        }

        const std::size_t frameSize = getFrameSize(header);

        if (availableSize() < frameSize)
        {
            break;
        }

        result.frames.push_back(
            extractFrame(
                header,
                frameSize));
    }

    compactBuffer();
    return result;
}

void MessageFramer::reset()
{
    _buffer.clear();
    _readOffset = 0;
}

protocol::ST_MsgHeader MessageFramer::peekHeader() const
{
    protocol::ST_MsgHeader header{};

    std::memcpy(
        &header,
        _buffer.data() + _readOffset,
        sizeof(header));

    return header;
}

FramingError MessageFramer::validateHeader(
    const protocol::ST_MsgHeader& header
) const
{
    if (!isKnownMessageId(header.message_Id))
    {
        return FramingError::UNKNOWN_MESSAGE_ID;
    }

    const std::size_t frameSize = getFrameSize(header);

    if (frameSize < sizeof(protocol::ST_MsgHeader) ||
        frameSize > _maximumFrameSize)
    {
        return FramingError::INVALID_FRAME_SIZE;
    }

    return FramingError::NONE;
}

bool MessageFramer::isKnownMessageId(
    uint16_t messageId
) const
{
    switch (static_cast<protocol::MessageId>(messageId))
    {
    case protocol::MessageId::OperatorControl:
    case protocol::MessageId::TrackInformation:
    case protocol::MessageId::RadarAttitudeToConsole:
    case protocol::MessageId::TrackDeletion:
    case protocol::MessageId::BeamTransmit:
    case protocol::MessageId::PlotInformation:
    case protocol::MessageId::RadarAttitudeFromSimulator:
        return true;
    }

    return false;
}

std::size_t MessageFramer::getFrameSize(
    const protocol::ST_MsgHeader& header
) const
{
    return static_cast<std::size_t>(header.block_size);
}

MessageFrame MessageFramer::extractFrame(
    const protocol::ST_MsgHeader& header,
    std::size_t frameSize
)
{
    MessageFrame frame;
    frame.header = header;

    const auto frameBegin =
        _buffer.cbegin() +
        static_cast<std::ptrdiff_t>(_readOffset);

    const auto frameEnd =
        frameBegin +
        static_cast<std::ptrdiff_t>(frameSize);

    frame.bytes.assign(frameBegin, frameEnd);
    _readOffset += frameSize;

    return frame;
}

std::size_t MessageFramer::availableSize() const
{
    return _buffer.size() - _readOffset;
}

void MessageFramer::compactBuffer()
{
    if (_readOffset == 0)
    {
        return;
    }

    if (_readOffset == _buffer.size())
    {
        _buffer.clear();
        _readOffset = 0;
        return;
    }

    if (_readOffset >= _buffer.size() / 2)
    {
        _buffer.erase(
            _buffer.begin(),
            _buffer.begin() +
                static_cast<std::ptrdiff_t>(_readOffset));

        _readOffset = 0;
    }
}

} // namespace comm
