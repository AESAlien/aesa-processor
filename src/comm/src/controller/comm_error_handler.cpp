#include <controller/comm_error_handler.hpp>

#include <comm/controller/comm_error.hpp>

#include <iostream>
#include <mutex>

namespace comm
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

std::mutex& logMutex()
{
    static std::mutex mutex;
    return mutex;
}

void reportNetworkFailure(
    const char* action,
    NetworkOperation operation,
    int errorCode
) noexcept
{
    try
    {
        std::lock_guard<std::mutex> lock(logMutex());

        std::cerr
            << "[CommController] Failed to "
            << action
            << ": operation="
            << operationName(operation)
            << ", systemError="
            << errorCode
            << '\n';
    }
    catch (...)
    {
        // A logging failure must not replace the communication failure.
    }
}

void reportProtocolFailure(
    const char* stage,
    NetworkPeer peer,
    const char* error
) noexcept
{
    try
    {
        std::lock_guard<std::mutex> lock(logMutex());

        std::cerr
            << "[CommController] Failed to process received data: stage="
            << stage
            << ", peer="
            << peerName(peer)
            << ", error="
            << error
            << '\n';
    }
    catch (...)
    {
        // A logging failure must not replace the communication failure.
    }
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
    reportNetworkFailure(
        "open server",
        result.operation,
        result.errorCode);
    throw ServerOpenError(
        "Communication server could not be opened");
}

void throwPollFailure(const PollResult& result)
{
    if (isConnectionOperation(result.failedOperation))
    {
        reportNetworkFailure(
            "accept connection",
            result.failedOperation,
            result.errorCode);
        throw ConnectionError(
            "Communication connection could not be established");
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
    reportNetworkFailure(
        "receive data",
        operation,
        errorCode);
    throw ReceiveError(
        "Communication data could not be received");
}

void throwFramingFailure(
    NetworkPeer peer,
    FramingError error
)
{
    reportProtocolFailure(
        "framing",
        peer,
        framingErrorName(error));
    throw ReceiveError(
        "Communication data could not be received");
}

void throwDecodingFailure(
    NetworkPeer peer,
    DecodingError error
)
{
    reportProtocolFailure(
        "decoding",
        peer,
        decodingErrorName(error));
    throw ReceiveError(
        "Communication data could not be received");
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
} // namespace comm
