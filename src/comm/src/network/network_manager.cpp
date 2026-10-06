#include "network_manager.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#ifdef ERROR
#undef ERROR
#endif
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <chrono>
#include <limits>
#include <utility>

namespace comm
{

namespace
{

#ifdef _WIN32
constexpr SocketHandle INVALID_SOCKET_HANDLE =
    static_cast<SocketHandle>(INVALID_SOCKET);
using SocketLength = int;
#else
constexpr SocketHandle INVALID_SOCKET_HANDLE = -1;
using SocketLength = socklen_t;
#endif

constexpr int RECEIVE_BUFFER_SIZE = 4096;

timeval makeTimeval(std::chrono::microseconds timeout)
{
    using Seconds = std::chrono::seconds;

    const auto seconds = std::chrono::duration_cast<Seconds>(timeout);
    const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        timeout - seconds);

    timeval value{};
    value.tv_sec = static_cast<decltype(value.tv_sec)>(seconds.count());
    value.tv_usec = static_cast<decltype(value.tv_usec)>(microseconds.count());
    return value;
}

std::chrono::microseconds remainingTime(
    std::chrono::steady_clock::time_point deadline
)
{
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline)
    {
        return std::chrono::microseconds{0};
    }

    return std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
}

int lastSocketError()
{
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

bool isInterrupted(int errorCode)
{
#ifdef _WIN32
    return errorCode == WSAEINTR;
#else
    return errorCode == EINTR;
#endif
}

bool isWouldBlock(int errorCode)
{
#ifdef _WIN32
    return errorCode == WSAEWOULDBLOCK;
#else
    return errorCode == EAGAIN || errorCode == EWOULDBLOCK;
#endif
}

int closeNativeSocket(SocketHandle socket)
{
#ifdef _WIN32
    return closesocket(socket);
#else
    return close(socket);
#endif
}

void shutdownNativeSocket(SocketHandle socket)
{
#ifdef _WIN32
    shutdown(socket, SD_BOTH);
#else
    shutdown(socket, SHUT_RDWR);
#endif
}

int setNonBlocking(SocketHandle socket)
{
#ifdef _WIN32
    u_long enabled = 1;
    return ioctlsocket(socket, FIONBIO, &enabled);
#else
    int flags = fcntl(socket, F_GETFL, 0);
    if (flags < 0)
    {
        return -1;
    }
    return fcntl(socket, F_SETFL, flags | O_NONBLOCK);
#endif
}

int socketSelect(
    SocketHandle maximumSocket,
    fd_set* readSockets,
    fd_set* writeSockets,
    timeval* timeout
)
{
#ifdef _WIN32
    static_cast<void>(maximumSocket);
    return select(0, readSockets, writeSockets, nullptr, timeout);
#else
    return select(
        maximumSocket + 1,
        readSockets,
        writeSockets,
        nullptr,
        timeout);
#endif
}

int sendBytes(SocketHandle socket, const std::uint8_t* data, int size)
{
#ifdef _WIN32
    return ::send(
        socket,
        reinterpret_cast<const char*>(data),
        size,
        0);
#else
    return static_cast<int>(::send(socket, data, size, MSG_NOSIGNAL));
#endif
}

int receiveBytes(SocketHandle socket, std::uint8_t* buffer, int size)
{
#ifdef _WIN32
    return recv(socket, reinterpret_cast<char*>(buffer), size, 0);
#else
    return static_cast<int>(recv(socket, buffer, size, 0));
#endif
}

} // namespace

NetworkManager::NetworkManager(
    NetworkEndpoint serverEndpoint,
    NetworkEndpoint operatorConsoleEndpoint,
    NetworkEndpoint scenarioSimulatorEndpoint
) : _serverEndpoint(std::move(serverEndpoint)),
    _operatorConsoleEndpoint(std::move(operatorConsoleEndpoint)),
    _scenarioSimulatorEndpoint(std::move(scenarioSimulatorEndpoint)),
    _socketApiStarted(false),
    _listenSocket(INVALID_SOCKET_HANDLE),
    _operatorConsoleSocket(INVALID_SOCKET_HANDLE),
    _scenarioSimulatorSocket(INVALID_SOCKET_HANDLE)
{
}

NetworkManager::~NetworkManager()
{
    stop();
}

OperationResult NetworkManager::validateConfiguration()
{
    _serverAddress.reset();
    _operatorConsoleAddress.reset();
    _scenarioSimulatorAddress.reset();

    auto parseEndpoint = [](
        const NetworkEndpoint& endpoint,
        std::optional<std::uint32_t>& parsedAddress,
        bool allowDisabled
    ) {
        bool disabled = endpoint.address.empty() && endpoint.port == 0;
        if (disabled)
        {
            return allowDisabled;
        }

        if (endpoint.address.empty() || endpoint.port == 0)
        {
            return false;
        }

        in_addr address{};
        int parseResult = inet_pton(AF_INET, endpoint.address.c_str(), &address);
        if (parseResult != 1)
        {
            return false;
        }

        parsedAddress = address.s_addr;
        return true;
    };

    if (!parseEndpoint(_serverEndpoint, _serverAddress, false) ||
        !parseEndpoint(
            _operatorConsoleEndpoint,
            _operatorConsoleAddress,
            true) ||
        !parseEndpoint(
            _scenarioSimulatorEndpoint,
            _scenarioSimulatorAddress,
            true) ||
        (!_operatorConsoleAddress.has_value() &&
         !_scenarioSimulatorAddress.has_value()))
    {
        return
        {
            OperationStatus::INVALID_ARGUMENT,
            NetworkOperation::CONFIGURATION,
            0
        };
    }

    if (_operatorConsoleAddress.has_value() &&
        _scenarioSimulatorAddress.has_value() &&
        _operatorConsoleAddress == _scenarioSimulatorAddress &&
        _operatorConsoleEndpoint.port == _scenarioSimulatorEndpoint.port)
    {
        return
        {
            OperationStatus::INVALID_ARGUMENT,
            NetworkOperation::CONFIGURATION,
            0
        };
    }

    return {};
}

OperationResult NetworkManager::start()
{
    if (_listenSocket != INVALID_SOCKET_HANDLE)
    {
        return {};
    }

    OperationResult configuration = validateConfiguration();
    if (configuration.status != OperationStatus::SUCCESS)
    {
        return configuration;
    }

#ifdef _WIN32
    WSADATA socketApiData{};
    int startupError = WSAStartup(MAKEWORD(2, 2), &socketApiData);
    if (startupError != 0)
    {
        return
        {
            OperationStatus::ERROR,
            NetworkOperation::SOCKET_API_STARTUP,
            startupError
        };
    }
#endif
    _socketApiStarted = true;

    auto failStart = [this](NetworkOperation operation, int errorCode) {
        closeSocket(_listenSocket);
#ifdef _WIN32
        WSACleanup();
#endif
        _socketApiStarted = false;
        return OperationResult{OperationStatus::ERROR, operation, errorCode};
    };

    _listenSocket = static_cast<SocketHandle>(socket(AF_INET, SOCK_STREAM, 0));
    if (_listenSocket == INVALID_SOCKET_HANDLE)
    {
        return failStart(NetworkOperation::CREATE_SOCKET, lastSocketError());
    }

    int reuseAddress = 1;
    if (setsockopt(
            _listenSocket,
            SOL_SOCKET,
            SO_REUSEADDR,
#ifdef _WIN32
            reinterpret_cast<const char*>(&reuseAddress),
#else
            &reuseAddress,
#endif
            sizeof(reuseAddress)) < 0)
    {
        return failStart(NetworkOperation::SET_SOCKET_OPTION, lastSocketError());
    }

    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = *_serverAddress;
    serverAddress.sin_port = htons(_serverEndpoint.port);

    if (bind(
            _listenSocket,
            reinterpret_cast<sockaddr*>(&serverAddress),
            sizeof(serverAddress)) < 0)
    {
        return failStart(NetworkOperation::BIND, lastSocketError());
    }

    if (listen(_listenSocket, 2) < 0)
    {
        return failStart(NetworkOperation::LISTEN, lastSocketError());
    }

    if (setNonBlocking(_listenSocket) < 0)
    {
        return failStart(NetworkOperation::SET_SOCKET_OPTION, lastSocketError());
    }

    return {};
}

PollResult NetworkManager::poll(std::chrono::milliseconds timeout)
{
    if (_listenSocket == INVALID_SOCKET_HANDLE)
    {
        PollResult result;
        result.status = PollStatus::NOT_STARTED;
        return result;
    }

    if (timeout.count() < 0)
    {
        PollResult result;
        result.status = PollStatus::INVALID_ARGUMENT;
        return result;
    }

    const Clock::time_point deadline = Clock::now() + timeout;

    while (true)
    {
        fd_set readSockets;
        FD_ZERO(&readSockets);
        FD_SET(_listenSocket, &readSockets);

        SocketHandle maximumSocket = _listenSocket;
        const SocketHandle operatorConsoleSocket = _operatorConsoleSocket;
        const SocketHandle scenarioSimulatorSocket = _scenarioSimulatorSocket;

        if (operatorConsoleSocket != INVALID_SOCKET_HANDLE)
        {
            FD_SET(operatorConsoleSocket, &readSockets);
            if (operatorConsoleSocket > maximumSocket)
            {
                maximumSocket = operatorConsoleSocket;
            }
        }

        if (scenarioSimulatorSocket != INVALID_SOCKET_HANDLE)
        {
            FD_SET(scenarioSimulatorSocket, &readSockets);
            if (scenarioSimulatorSocket > maximumSocket)
            {
                maximumSocket = scenarioSimulatorSocket;
            }
        }

        timeval selectTimeout = makeTimeval(remainingTime(deadline));
        int selectedCount = socketSelect(
            maximumSocket,
            &readSockets,
            nullptr,
            &selectTimeout);

        if (selectedCount == 0)
        {
            PollResult result;
            result.status = PollStatus::TIMEOUT;
            return result;
        }

        if (selectedCount < 0)
        {
            int errorCode = lastSocketError();
            if (isInterrupted(errorCode) && Clock::now() < deadline)
            {
                continue;
            }

            PollResult result;
            result.status = PollStatus::ERROR;
            result.failedOperation = NetworkOperation::SELECT;
            result.errorCode = errorCode;
            return result;
        }

        PollResult result;
        result.status = PollStatus::EVENTS;

        if (FD_ISSET(_listenSocket, &readSockets))
        {
            if (!acceptConnection(
                    result.events,
                    result.failedOperation,
                    result.errorCode))
            {
                result.status = PollStatus::ERROR;
                result.events.clear();
                return result;
            }
        }

        if (operatorConsoleSocket != INVALID_SOCKET_HANDLE &&
            operatorConsoleSocket == _operatorConsoleSocket &&
            FD_ISSET(operatorConsoleSocket, &readSockets))
        {
            receiveFromSocket(
                operatorConsoleSocket,
                NetworkPeer::OPERATOR_CONSOLE,
                result.events);
        }

        if (scenarioSimulatorSocket != INVALID_SOCKET_HANDLE &&
            scenarioSimulatorSocket == _scenarioSimulatorSocket &&
            FD_ISSET(scenarioSimulatorSocket, &readSockets))
        {
            receiveFromSocket(
                scenarioSimulatorSocket,
                NetworkPeer::SCENARIO_SIMULATOR,
                result.events);
        }

        if (!result.events.empty())
        {
            return result;
        }

        if (Clock::now() >= deadline)
        {
            result.status = PollStatus::TIMEOUT;
            return result;
        }
    }
}

SendResult NetworkManager::send(
    NetworkPeer peer,
    const std::vector<std::uint8_t>& data,
    std::chrono::milliseconds timeout
)
{
    SendResult result;

    if (_listenSocket == INVALID_SOCKET_HANDLE)
    {
        result.status = SendStatus::NOT_STARTED;
        return result;
    }

    if (data.empty() || timeout.count() < 0)
    {
        result.status = SendStatus::INVALID_ARGUMENT;
        return result;
    }

    SocketHandle socket = getSocket(peer);
    if (socket == INVALID_SOCKET_HANDLE)
    {
        result.status = SendStatus::NOT_CONNECTED;
        return result;
    }

    const Clock::time_point deadline = Clock::now() + timeout;

    while (result.bytesSent < data.size())
    {
        std::size_t remainingSize = data.size() - result.bytesSent;
        int sendSize = remainingSize > static_cast<std::size_t>(
            std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : static_cast<int>(remainingSize);

        int sentSize = sendBytes(
            socket,
            data.data() + result.bytesSent,
            sendSize);

        if (sentSize > 0)
        {
            result.bytesSent += static_cast<std::size_t>(sentSize);
            continue;
        }

        if (sentSize == 0)
        {
            closeSocket(socket);
            setSocket(peer, INVALID_SOCKET_HANDLE);
            result.status = SendStatus::CONNECTION_LOST;
            result.failedOperation = NetworkOperation::SEND;
            return result;
        }

        int errorCode = lastSocketError();
        if (isInterrupted(errorCode))
        {
            continue;
        }

        if (!isWouldBlock(errorCode))
        {
            closeSocket(socket);
            setSocket(peer, INVALID_SOCKET_HANDLE);
            result.status = SendStatus::CONNECTION_LOST;
            result.failedOperation = NetworkOperation::SEND;
            result.errorCode = errorCode;
            return result;
        }

        std::chrono::microseconds remaining = remainingTime(deadline);
        if (remaining.count() == 0)
        {
            result.status = SendStatus::TIMEOUT;
            return result;
        }

        fd_set writeSockets;
        FD_ZERO(&writeSockets);
        FD_SET(socket, &writeSockets);
        timeval selectTimeout = makeTimeval(remaining);

        int selectedCount = socketSelect(
            socket,
            nullptr,
            &writeSockets,
            &selectTimeout);

        if (selectedCount > 0)
        {
            continue;
        }

        if (selectedCount == 0)
        {
            result.status = SendStatus::TIMEOUT;
            return result;
        }

        errorCode = lastSocketError();
        if (isInterrupted(errorCode) && Clock::now() < deadline)
        {
            continue;
        }

        result.status = SendStatus::ERROR;
        result.failedOperation = NetworkOperation::SELECT;
        result.errorCode = errorCode;
        return result;
    }

    result.status = SendStatus::SUCCESS;
    return result;
}

bool NetworkManager::isConnected(NetworkPeer peer) const
{
    return getSocket(peer) != INVALID_SOCKET_HANDLE;
}

OperationResult NetworkManager::disconnect(NetworkPeer peer)
{
    if (_listenSocket == INVALID_SOCKET_HANDLE)
    {
        return
        {
            OperationStatus::NOT_STARTED,
            NetworkOperation::NONE,
            0
        };
    }

    SocketHandle socket = getSocket(peer);
    if (socket == INVALID_SOCKET_HANDLE)
    {
        return
        {
            OperationStatus::NOT_CONNECTED,
            NetworkOperation::NONE,
            0
        };
    }

    int errorCode = closeSocket(socket);
    setSocket(peer, INVALID_SOCKET_HANDLE);
    if (errorCode != 0)
    {
        return
        {
            OperationStatus::ERROR,
            NetworkOperation::CLOSE,
            errorCode
        };
    }

    return {};
}

OperationResult NetworkManager::stop()
{
    int firstError = 0;

    int operatorError = closeSocket(_operatorConsoleSocket);
    if (operatorError != 0)
    {
        firstError = operatorError;
    }

    int simulatorError = closeSocket(_scenarioSimulatorSocket);
    if (firstError == 0 && simulatorError != 0)
    {
        firstError = simulatorError;
    }

    int listenerError = closeSocket(_listenSocket);
    if (firstError == 0 && listenerError != 0)
    {
        firstError = listenerError;
    }

    int cleanupError = 0;
    if (_socketApiStarted)
    {
#ifdef _WIN32
        if (WSACleanup() == SOCKET_ERROR)
        {
            cleanupError = lastSocketError();
        }
#endif
        _socketApiStarted = false;
    }

    if (firstError != 0)
    {
        return
        {
            OperationStatus::ERROR,
            NetworkOperation::CLOSE,
            firstError
        };
    }

    if (cleanupError != 0)
    {
        return
        {
            OperationStatus::ERROR,
            NetworkOperation::SOCKET_API_CLEANUP,
            cleanupError
        };
    }

    return {};
}

bool NetworkManager::acceptConnection(
    std::vector<NetworkEvent>& events,
    NetworkOperation& failedOperation,
    int& errorCode
)
{
    sockaddr_in clientAddress{};
    SocketLength clientAddressSize = sizeof(clientAddress);

    SocketHandle clientSocket = static_cast<SocketHandle>(accept(
        _listenSocket,
        reinterpret_cast<sockaddr*>(&clientAddress),
        &clientAddressSize));

    if (clientSocket == INVALID_SOCKET_HANDLE)
    {
        errorCode = lastSocketError();
        if (isWouldBlock(errorCode))
        {
            errorCode = 0;
            return true;
        }

        failedOperation = NetworkOperation::ACCEPT;
        return false;
    }

    if (setNonBlocking(clientSocket) < 0)
    {
        errorCode = lastSocketError();
        failedOperation = NetworkOperation::SET_SOCKET_OPTION;
        closeNativeSocket(clientSocket);
        return false;
    }

    const std::uint16_t clientPort = ntohs(clientAddress.sin_port);
    std::optional<NetworkPeer> peer;
    if (_operatorConsoleAddress.has_value() &&
        clientAddress.sin_addr.s_addr == *_operatorConsoleAddress &&
        clientPort == _operatorConsoleEndpoint.port)
    {
        peer = NetworkPeer::OPERATOR_CONSOLE;
    }
    else if (_scenarioSimulatorAddress.has_value() &&
             clientAddress.sin_addr.s_addr == *_scenarioSimulatorAddress &&
             clientPort == _scenarioSimulatorEndpoint.port)
    {
        peer = NetworkPeer::SCENARIO_SIMULATOR;
    }

    if (!peer.has_value())
    {
        char addressBuffer[INET_ADDRSTRLEN]{};
        const char* address = inet_ntop(
            AF_INET,
            &clientAddress.sin_addr,
            addressBuffer,
            sizeof(addressBuffer));

        if (address == nullptr)
        {
            errorCode = lastSocketError();
            failedOperation = NetworkOperation::ADDRESS_CONVERSION;
            closeNativeSocket(clientSocket);
            return false;
        }

        ConnectionRejectedEvent event;
        event.remoteAddress = address;
        event.remotePort = clientPort;
        events.emplace_back(std::move(event));
        closeNativeSocket(clientSocket);
        return true;
    }

    SocketHandle previousSocket = getSocket(*peer);
    bool reconnected = previousSocket != INVALID_SOCKET_HANDLE;
    closeSocket(previousSocket);
    setSocket(*peer, clientSocket);

    ConnectionEvent event;
    event.peer = *peer;
    event.state = reconnected
        ? ConnectionState::RECONNECTED
        : ConnectionState::CONNECTED;
    events.emplace_back(event);
    return true;
}

void NetworkManager::receiveFromSocket(
    SocketHandle socket,
    NetworkPeer peer,
    std::vector<NetworkEvent>& events
)
{
    std::uint8_t buffer[RECEIVE_BUFFER_SIZE];
    int receivedSize = receiveBytes(socket, buffer, sizeof(buffer));

    if (receivedSize > 0)
    {
        DataReceivedEvent event;
        event.peer = peer;
        event.bytes.assign(buffer, buffer + receivedSize);
        events.emplace_back(std::move(event));
        return;
    }

    ConnectionEvent event;
    event.peer = peer;
    event.state = ConnectionState::DISCONNECTED;

    if (receivedSize == 0)
    {
        event.disconnectReason = DisconnectReason::REMOTE_CLOSED;
    }
    else
    {
        event.errorCode = lastSocketError();
        if (isWouldBlock(event.errorCode))
        {
            return;
        }
        event.disconnectReason = DisconnectReason::RECEIVE_ERROR;
    }

    SocketHandle currentSocket = getSocket(peer);
    closeSocket(currentSocket);
    setSocket(peer, INVALID_SOCKET_HANDLE);
    events.emplace_back(event);
}

SocketHandle NetworkManager::getSocket(NetworkPeer peer) const
{
    switch (peer)
    {
    case NetworkPeer::OPERATOR_CONSOLE:
        return _operatorConsoleSocket;

    case NetworkPeer::SCENARIO_SIMULATOR:
        return _scenarioSimulatorSocket;
    }

    return INVALID_SOCKET_HANDLE;
}

void NetworkManager::setSocket(NetworkPeer peer, SocketHandle socket)
{
    switch (peer)
    {
    case NetworkPeer::OPERATOR_CONSOLE:
        _operatorConsoleSocket = socket;
        return;

    case NetworkPeer::SCENARIO_SIMULATOR:
        _scenarioSimulatorSocket = socket;
        return;
    }
}

int NetworkManager::closeSocket(SocketHandle& socket)
{
    if (socket == INVALID_SOCKET_HANDLE)
    {
        return 0;
    }

    SocketHandle socketToClose = socket;
    socket = INVALID_SOCKET_HANDLE;

    shutdownNativeSocket(socketToClose);
    if (closeNativeSocket(socketToClose) < 0)
    {
        return lastSocketError();
    }

    return 0;
}

} // namespace comm
