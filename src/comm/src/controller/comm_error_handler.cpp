#include <controller/comm_error_handler.hpp>

#include <comm/error/comm_error.hpp>

#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

namespace aesa_processor::comm
{
namespace controller_error
{
namespace
{

const char* operationName(NetworkOperation operation)
{
    switch (operation)
    {
    case NetworkOperation::CONFIGURATION:
        return "configuration";

    case NetworkOperation::SOCKET_API_STARTUP:
        return "socket-api-startup";

    case NetworkOperation::CREATE_SOCKET:
        return "create-socket";

    case NetworkOperation::SET_SOCKET_OPTION:
        return "set-socket-option";

    case NetworkOperation::BIND:
        return "bind";

    case NetworkOperation::LISTEN:
        return "listen";

    case NetworkOperation::SELECT:
        return "select";

    case NetworkOperation::ACCEPT:
        return "accept";

    case NetworkOperation::ADDRESS_CONVERSION:
        return "address-conversion";

    case NetworkOperation::RECEIVE:
        return "receive";

    case NetworkOperation::SEND:
        return "send";

    case NetworkOperation::CLOSE:
        return "close";

    case NetworkOperation::SOCKET_API_CLEANUP:
        return "socket-api-cleanup";

    case NetworkOperation::NONE:
        return "none";
    }

    return "unknown";
}

const char* peerName(NetworkPeer peer)
{
    switch (peer)
    {
    case NetworkPeer::OPERATOR_CONSOLE:
        return "operator-console";

    case NetworkPeer::SCENARIO_SIMULATOR:
        return "scenario-simulator";
    }

    return "unknown";
}

const char* framingErrorName(FramingError error)
{
    switch (error)
    {
    case FramingError::NONE:
        return "none";

    case FramingError::UNKNOWN_MESSAGE_ID:
        return "unknown-message-id";

    case FramingError::INVALID_FRAME_SIZE:
        return "invalid-frame-size";
    }

    return "unknown";
}

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

const char* sendStatusName(SendStatus status)
{
    switch (status)
    {
    case SendStatus::SUCCESS:
        return "success";

    case SendStatus::TIMEOUT:
        return "timeout";

    case SendStatus::NOT_STARTED:
        return "not-started";

    case SendStatus::NOT_CONNECTED:
        return "not-connected";

    case SendStatus::INVALID_ARGUMENT:
        return "invalid-argument";

    case SendStatus::CONNECTION_LOST:
        return "connection-lost";

    case SendStatus::ERROR:
        return "error";
    }

    return "unknown";
}

std::mutex& logMutex()
{
    static std::mutex mutex;
    return mutex;
}

std::string makeNetworkFailureMessage(
    const char* message,
    NetworkOperation operation,
    int errorCode
)
{
    std::ostringstream stream;
    stream
        << message
        << ": operation="
        << operationName(operation)
        << ", systemError="
        << errorCode;
    return stream.str();
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

std::string makeSendFailureMessage(
    NetworkPeer peer,
    const SendResult& result
)
{
    std::ostringstream stream;
    stream
        << "Communication data could not be sent"
        << ": peer="
        << peerName(peer)
        << ", status="
        << sendStatusName(result.status)
        << ", operation="
        << operationName(result.failedOperation)
        << ", systemError="
        << result.errorCode
        << ", bytesSent="
        << result.bytesSent;
    return stream.str();
}

bool isConnectionOperation(NetworkOperation operation)
{
    return operation == NetworkOperation::ACCEPT ||
        operation == NetworkOperation::SET_SOCKET_OPTION ||
        operation == NetworkOperation::ADDRESS_CONVERSION;
}

} // namespace

void throwServerOpenFailure(const OperationResult& result)
{
    throw ServerOpenError(
        makeNetworkFailureMessage(
            "Communication server could not be opened",
            result.operation,
            result.errorCode));
}

void throwPollFailure(const PollResult& result)
{
    if (isConnectionOperation(result.failedOperation))
    {
        throw ConnectionError(
            makeNetworkFailureMessage(
                "Communication connection could not be established",
                result.failedOperation,
                result.errorCode));
    }

    throwReceiveFailure(
        result.failedOperation,
        result.errorCode);
}

void throwReceiveFailure(
    NetworkOperation operation,
    int errorCode
)
{
    throw ReceiveError(
        makeNetworkFailureMessage(
            "Communication data could not be received",
            operation,
            errorCode));
}

void throwFramingFailure(
    NetworkPeer peer,
    FramingError error
)
{
    throw ReceiveError(
        makeProtocolFailureMessage(
            "Communication data could not be received",
            "framing",
            peer,
            framingErrorName(error)));
}

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

void throwSendFailure(
    NetworkPeer peer,
    const SendResult& result
)
{
    throw SendError(makeSendFailureMessage(peer, result));
}

void reportRejectedConnection(
    const ConnectionRejectedEvent& event
) noexcept
{
    try
    {
        std::lock_guard<std::mutex> lock(logMutex());

        std::cerr
            << "[CommController] Rejected unknown connection: remoteAddress="
            << event.remoteAddress
            << ", remotePort="
            << event.remotePort
            << '\n';
    }
    catch (...)
    {
        // A logging failure must not interrupt communication processing.
    }
}

} // namespace controller_error
} // namespace aesa_processor::comm
