#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace comm
{

struct NetworkEndpoint
{
    std::string address;
    std::uint16_t port = 0;
};

enum class NetworkPeer
{
    OPERATOR_CONSOLE,
    SCENARIO_SIMULATOR
};

enum class NetworkOperation
{
    NONE,
    CONFIGURATION,
    SOCKET_API_STARTUP,
    CREATE_SOCKET,
    SET_SOCKET_OPTION,
    BIND,
    LISTEN,
    SELECT,
    ACCEPT,
    ADDRESS_CONVERSION,
    RECEIVE,
    SEND,
    CLOSE,
    SOCKET_API_CLEANUP
};

enum class OperationStatus
{
    SUCCESS,
    NOT_STARTED,
    NOT_CONNECTED,
    INVALID_ARGUMENT,
    ERROR
};

struct OperationResult
{
    OperationStatus status = OperationStatus::SUCCESS;
    NetworkOperation operation = NetworkOperation::NONE;
    int errorCode = 0;
};

enum class ConnectionState
{
    CONNECTED,
    RECONNECTED,
    DISCONNECTED
};

enum class DisconnectReason
{
    NONE,
    REMOTE_CLOSED,
    RECEIVE_ERROR
};

struct ConnectionEvent
{
    NetworkPeer peer;
    ConnectionState state;
    DisconnectReason disconnectReason = DisconnectReason::NONE;
    int errorCode = 0;
};

struct DataReceivedEvent
{
    NetworkPeer peer;
    std::vector<std::uint8_t> bytes;
};

struct ConnectionRejectedEvent
{
    std::string remoteAddress;
    std::uint16_t remotePort = 0;
};

using NetworkEvent = std::variant<
    ConnectionEvent,
    DataReceivedEvent,
    ConnectionRejectedEvent
>;

enum class PollStatus
{
    EVENTS,
    TIMEOUT,
    NOT_STARTED,
    INVALID_ARGUMENT,
    ERROR
};

struct PollResult
{
    PollStatus status = PollStatus::TIMEOUT;
    std::vector<NetworkEvent> events;
    NetworkOperation failedOperation = NetworkOperation::NONE;
    int errorCode = 0;
};

enum class SendStatus
{
    SUCCESS,
    TIMEOUT,
    NOT_STARTED,
    NOT_CONNECTED,
    INVALID_ARGUMENT,
    CONNECTION_LOST,
    ERROR
};

struct SendResult
{
    SendStatus status = SendStatus::SUCCESS;
    std::size_t bytesSent = 0;
    NetworkOperation failedOperation = NetworkOperation::NONE;
    int errorCode = 0;
};

} // namespace comm
