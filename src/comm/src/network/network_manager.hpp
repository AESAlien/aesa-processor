#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace comm
{

#ifdef _WIN32
using SocketHandle = std::uintptr_t;
#else
using SocketHandle = int;
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

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
    ConnectionRejectedEvent>;

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

// IPv4 TCP server. Call every method from the same owning thread.
// An empty peer endpoint disables that peer. No message framing is performed.
class NetworkManager
{
public:
    NetworkManager(
        NetworkEndpoint serverEndpoint,
        NetworkEndpoint operatorConsoleEndpoint,
        NetworkEndpoint scenarioSimulatorEndpoint
    );

    ~NetworkManager();

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;
    NetworkManager(NetworkManager&&) = delete;
    NetworkManager& operator=(NetworkManager&&) = delete;

    OperationResult start();

    PollResult poll(std::chrono::milliseconds timeout);

    SendResult send(
        NetworkPeer peer,
        const std::vector<std::uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );

    bool isConnected(NetworkPeer peer) const;
    OperationResult disconnect(NetworkPeer peer);
    OperationResult stop();

private:
    using Clock = std::chrono::steady_clock;

    OperationResult validateConfiguration();
    bool acceptConnection(
        std::vector<NetworkEvent>& events,
        NetworkOperation& failedOperation,
        int& errorCode
    );
    void receiveFromSocket(
        SocketHandle socket,
        NetworkPeer peer,
        std::vector<NetworkEvent>& events
    );

    SocketHandle getSocket(NetworkPeer peer) const;
    void setSocket(NetworkPeer peer, SocketHandle socket);
    int closeSocket(SocketHandle& socket);

    NetworkEndpoint _serverEndpoint;
    NetworkEndpoint _operatorConsoleEndpoint;
    NetworkEndpoint _scenarioSimulatorEndpoint;
    std::optional<std::uint32_t> _serverAddress;
    std::optional<std::uint32_t> _operatorConsoleAddress;
    std::optional<std::uint32_t> _scenarioSimulatorAddress;

    bool _socketApiStarted;
    SocketHandle _listenSocket;
    SocketHandle _operatorConsoleSocket;
    SocketHandle _scenarioSimulatorSocket;
};

} // namespace comm
