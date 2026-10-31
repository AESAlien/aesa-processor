#include <controller/comm_error_handler.hpp>

#include <comm/error/comm_error.hpp>

#include <sstream>
#include <string>

namespace comm
{
namespace controller_error
{
namespace
{

const char* decodingErrorName(DecodingError error);
const char* encodingErrorName(EncodingError error);

std::string makeProtocolFailureMessage(
    const char* message,
    const char* stage,
    NetworkPeer peer,
    const char* error
);

} // namespace

void throwDecodingFailure(
    NetworkPeer peer,
    DecodingError error
)
{
    throw ReceiveError(
        makeProtocolFailureMessage(
            "Communication data could not be received",
            "decoding",
            peer,
            decodingErrorName(error)));
}

void throwEncodingFailure(
    NetworkPeer peer,
    EncodingError error
)
{
    throw SendError(
        makeProtocolFailureMessage(
            "Communication data could not be sent",
            "encoding",
            peer,
            encodingErrorName(error)));
}

namespace
{

const char* decodingErrorName(DecodingError error)
{
    switch (error)
    {
    case DecodingError::NONE:
        return "missing-message";

    case DecodingError::UNSUPPORTED_MESSAGE_ID:
        return "unsupported-message-id";

    case DecodingError::UNSUPPORTED_VERSION:
        return "unsupported-version";

    case DecodingError::INVALID_MESSAGE_SIZE:
        return "invalid-message-size";

    case DecodingError::INVALID_ITEM_COUNT:
        return "invalid-item-count";

    case DecodingError::INVALID_FIELD_VALUE:
        return "invalid-field-value";
    }

    return "unknown";
}

const char* encodingErrorName(EncodingError error)
{
    switch (error)
    {
    case EncodingError::NONE:
        return "missing-bytes";

    case EncodingError::TOO_MANY_TRACKS:
        return "too-many-tracks";

    case EncodingError::TYPE_CONVERSION_ERROR:
        return "type-conversion-error";
    }

    return "unknown";
}

std::string makeProtocolFailureMessage(
    const char* message,
    const char* stage,
    NetworkPeer peer,
    const char* error
)
{
    std::ostringstream stream;
    stream
        << message
        << ": stage="
        << stage
        << ", peer="
        << peerName(peer)
        << ", error="
        << error;
    return stream.str();
}

} // namespace
} // namespace controller_error
} // namespace comm
