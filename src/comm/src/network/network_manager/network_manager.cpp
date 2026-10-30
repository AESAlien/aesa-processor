#include <network/network_manager/network_manager.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

namespace comm
{
namespace
{

constexpr int RECEIVE_BUFFER_SIZE = 4096;
constexpr int LISTEN_BACKLOG = 2;

std::chrono::microseconds remainingTime(
    std::chrono::steady_clock::time_point deadline
);

// An endpoint with neither address nor port is treated as disabled.
bool parseEndpoint(
    const NetworkEndpoint& endpoint,
    std::optional<uint32_t>& parsedAddress,
    bool allowDisabled
);

bool contains(
    const std::vector<SocketHandle>& sockets,
    SocketHandle socket
);

PollResult pollFailure(NetworkOperation operation, int errorCode);

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

    const int startupError = native_socket::startup();
    if (startupError != 0)
    {
        return
        {
            OperationStatus::ERROR,
            NetworkOperation::SOCKET_API_STARTUP,
            startupError
        };
    }
    _socketApiStarted = true;

    auto failStart = [this](NetworkOperation operation) {
        const int errorCode = native_socket::lastError();
        closeSocket(_listenSocket);
        native_socket::cleanup();
        _socketApiStarted = false;
        return OperationResult{OperationStatus::ERROR, operation, errorCode};
    };

    _listenSocket = native_socket::createTcpSocket();
    if (_listenSocket == INVALID_SOCKET_HANDLE)
    {
        return failStart(NetworkOperation::CREATE_SOCKET);
    }

    if (native_socket::setReuseAddress(_listenSocket) < 0)
    {
        return failStart(NetworkOperation::SET_SOCKET_OPTION);
    }

    if (native_socket::bindSocket(
            _listenSocket,
            *_serverAddress,
            _serverEndpoint.port) < 0)
    {
        return failStart(NetworkOperation::BIND);
    }

    if (native_socket::listenSocket(_listenSocket, LISTEN_BACKLOG) < 0)
    {
        return failStart(NetworkOperation::LISTEN);
    }

    if (native_socket::setNonBlocking(_listenSocket) < 0)
    {
        return failStart(NetworkOperation::SET_SOCKET_OPTION);
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
        int closeError = closeRequestedSockets();
        if (closeError != 0)
        {
            return pollFailure(NetworkOperation::CLOSE, closeError);
        }

        const SocketHandle operatorConsoleSocket = _operatorConsoleSocket;
        const SocketHandle scenarioSimulatorSocket = _scenarioSimulatorSocket;

        std::vector<SocketHandle> sockets{_listenSocket};
        if (operatorConsoleSocket != INVALID_SOCKET_HANDLE)
        {
            sockets.push_back(operatorConsoleSocket);
        }
        if (scenarioSimulatorSocket != INVALID_SOCKET_HANDLE)
        {
            sockets.push_back(scenarioSimulatorSocket);
        }

        std::vector<SocketHandle> readySockets;
        const int selectedCount = native_socket::waitReadable(
            sockets,
            remainingTime(deadline),
            readySockets);

        closeError = closeRequestedSockets();
        if (closeError != 0)
        {
            return pollFailure(NetworkOperation::CLOSE, closeError);
        }

        if (selectedCount == 0)
        {
            PollResult result;
            result.status = PollStatus::TIMEOUT;
            return result;
        }

        if (selectedCount < 0)
        {
            const int errorCode = native_socket::lastError();
            if (native_socket::isInterrupted(errorCode) &&
                Clock::now() < deadline)
            {
                continue;
            }

            return pollFailure(NetworkOperation::SELECT, errorCode);
        }

        PollResult result;
        result.status = PollStatus::EVENTS;

        if (contains(readySockets, _listenSocket))
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

        // A socket replaced by a reconnection during this poll is skipped.
        if (operatorConsoleSocket != INVALID_SOCKET_HANDLE &&
            operatorConsoleSocket == _operatorConsoleSocket &&
            contains(readySockets, operatorConsoleSocket))
        {
            receiveFromSocket(
                operatorConsoleSocket,
                NetworkPeer::OPERATOR_CONSOLE,
                result.events);
        }

        if (scenarioSimulatorSocket != INVALID_SOCKET_HANDLE &&
            scenarioSimulatorSocket == _scenarioSimulatorSocket &&
            contains(readySockets, scenarioSimulatorSocket))
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
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
)
{
    SendResult result;

    if (data.empty() || timeout.count() < 0)
    {
        result.status = SendStatus::INVALID_ARGUMENT;
        return result;
    }

    PeerSynchro& synchro = getPeerSynchro(peer);
    std::lock_guard<std::mutex> lock(synchro.sendCloseMutex);

    if (_listenSocket == INVALID_SOCKET_HANDLE)
    {
        result.status = SendStatus::NOT_STARTED;
        return result;
    }

    SocketHandle socket = getSocket(peer);
    if (synchro.closeRequested.load() ||
        socket == INVALID_SOCKET_HANDLE)
    {
        result.status = SendStatus::NOT_CONNECTED;
        return result;
    }

    const Clock::time_point deadline = Clock::now() + timeout;

    // A partially sent message corrupts the stream, so the peer is closed.
    auto failSend = [&](SendStatus status) {
        if (result.bytesSent > 0)
        {
            synchro.closeRequested.store(true);
        }
        result.status = status;
        return result;
    };

    while (result.bytesSent < data.size())
    {
        std::size_t remainingSize = data.size() - result.bytesSent;
        int sendSize = remainingSize > static_cast<std::size_t>(
            std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : static_cast<int>(remainingSize);

        int sentSize = native_socket::sendBytes(
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
            synchro.closeRequested.store(true);
            result.status = SendStatus::CONNECTION_LOST;
            result.failedOperation = NetworkOperation::SEND;
            return result;
        }

        int errorCode = native_socket::lastError();
        if (native_socket::isInterrupted(errorCode))
        {
            continue;
        }

        if (!native_socket::isWouldBlock(errorCode))
        {
            synchro.closeRequested.store(true);
            result.status = SendStatus::CONNECTION_LOST;
            result.failedOperation = NetworkOperation::SEND;
            result.errorCode = errorCode;
            return result;
        }

        std::chrono::microseconds remaining = remainingTime(deadline);
        if (remaining.count() == 0)
        {
            return failSend(SendStatus::TIMEOUT);
        }

        int selectedCount = native_socket::waitWritable(socket, remaining);
        if (selectedCount > 0)
        {
            continue;
        }

        if (selectedCount == 0)
        {
            return failSend(SendStatus::TIMEOUT);
        }

        errorCode = native_socket::lastError();
        if (native_socket::isInterrupted(errorCode) &&
            Clock::now() < deadline)
        {
            continue;
        }

        result.failedOperation = NetworkOperation::SELECT;
        result.errorCode = errorCode;
        return failSend(SendStatus::ERROR);
    }

    result.status = SendStatus::SUCCESS;
    return result;
}

OperationResult NetworkManager::stop()
{
    _operatorConsoleSynchro.closeRequested.store(true);
    _scenarioSimulatorSynchro.closeRequested.store(true);
    std::scoped_lock lock(
        _operatorConsoleSynchro.sendCloseMutex,
        _scenarioSimulatorSynchro.sendCloseMutex);

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

    _operatorConsoleSynchro.closeRequested.store(false);
    _scenarioSimulatorSynchro.closeRequested.store(false);

    int cleanupError = 0;
    if (_socketApiStarted)
    {
        cleanupError = native_socket::cleanup();
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

OperationResult NetworkManager::validateConfiguration()
{
    _serverAddress.reset();
    _operatorConsoleAddress.reset();
    _scenarioSimulatorAddress.reset();

    const OperationResult invalidConfiguration
    {
        OperationStatus::INVALID_ARGUMENT,
        NetworkOperation::CONFIGURATION,
        0
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
        return invalidConfiguration;
    }

    if (_operatorConsoleAddress.has_value() &&
        _scenarioSimulatorAddress.has_value() &&
        _operatorConsoleAddress == _scenarioSimulatorAddress &&
        _operatorConsoleEndpoint.port == _scenarioSimulatorEndpoint.port)
    {
        return invalidConfiguration;
    }

    return {};
}

bool NetworkManager::acceptConnection(
    std::vector<NetworkEvent>& events,
    NetworkOperation& failedOperation,
    int& errorCode
)
{
    uint32_t clientAddress = 0;
    uint16_t clientPort = 0;
    SocketHandle clientSocket = native_socket::acceptSocket(
        _listenSocket,
        clientAddress,
        clientPort);

    if (clientSocket == INVALID_SOCKET_HANDLE)
    {
        errorCode = native_socket::lastError();
        if (native_socket::isWouldBlock(errorCode))
        {
            errorCode = 0;
            return true;
        }

        failedOperation = NetworkOperation::ACCEPT;
        return false;
    }

    if (native_socket::setNonBlocking(clientSocket) < 0)
    {
        errorCode = native_socket::lastError();
        failedOperation = NetworkOperation::SET_SOCKET_OPTION;
        native_socket::closeSocket(clientSocket);
        return false;
    }

    std::optional<NetworkPeer> peer;
    if (_operatorConsoleAddress == clientAddress &&
        clientPort == _operatorConsoleEndpoint.port)
    {
        peer = NetworkPeer::OPERATOR_CONSOLE;
    }
    else if (_scenarioSimulatorAddress == clientAddress &&
             clientPort == _scenarioSimulatorEndpoint.port)
    {
        peer = NetworkPeer::SCENARIO_SIMULATOR;
    }

    if (!peer.has_value())
    {
        const std::optional<std::string> address =
            native_socket::formatIpv4(clientAddress);

        if (!address.has_value())
        {
            errorCode = native_socket::lastError();
            failedOperation = NetworkOperation::ADDRESS_CONVERSION;
            native_socket::closeSocket(clientSocket);
            return false;
        }

        ConnectionRejectedEvent event;
        event.remoteAddress = *address;
        event.remotePort = clientPort;
        events.emplace_back(std::move(event));
        native_socket::closeSocket(clientSocket);
        return true;
    }

    PeerSynchro& synchro = getPeerSynchro(*peer);
    synchro.closeRequested.store(true);
    std::lock_guard<std::mutex> lock(synchro.sendCloseMutex);

    SocketHandle previousSocket = getSocket(*peer);
    bool reconnected = previousSocket != INVALID_SOCKET_HANDLE;
    closeSocket(previousSocket);
    setSocket(*peer, clientSocket);
    synchro.closeRequested.store(false);

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
    uint8_t buffer[RECEIVE_BUFFER_SIZE];
    int receivedSize = native_socket::receiveBytes(
        socket,
        buffer,
        sizeof(buffer));

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
        event.errorCode = native_socket::lastError();
        if (native_socket::isWouldBlock(event.errorCode))
        {
            return;
        }
        event.disconnectReason = DisconnectReason::RECEIVE_ERROR;
    }

    closeSocket(peer);
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

NetworkManager::PeerSynchro& NetworkManager::getPeerSynchro(
    NetworkPeer peer
) const
{
    switch (peer)
    {
    case NetworkPeer::OPERATOR_CONSOLE:
        return _operatorConsoleSynchro;

    case NetworkPeer::SCENARIO_SIMULATOR:
        return _scenarioSimulatorSynchro;
    }

    return _operatorConsoleSynchro;
}

int NetworkManager::closeRequestedSockets()
{
    int operatorError = 0;
    if (_operatorConsoleSynchro.closeRequested.load())
    {
        operatorError = closeSocket(NetworkPeer::OPERATOR_CONSOLE);
    }

    int simulatorError = 0;
    if (_scenarioSimulatorSynchro.closeRequested.load())
    {
        simulatorError = closeSocket(NetworkPeer::SCENARIO_SIMULATOR);
    }

    return operatorError != 0 ? operatorError : simulatorError;
}

int NetworkManager::closeSocket(NetworkPeer peer)
{
    PeerSynchro& synchro = getPeerSynchro(peer);
    synchro.closeRequested.store(true);
    std::lock_guard<std::mutex> lock(synchro.sendCloseMutex);

    SocketHandle socket = getSocket(peer);
    int errorCode = closeSocket(socket);
    setSocket(peer, socket);
    synchro.closeRequested.store(false);
    return errorCode;
}

int NetworkManager::closeSocket(SocketHandle& socket)
{
    if (socket == INVALID_SOCKET_HANDLE)
    {
        return 0;
    }

    SocketHandle socketToClose = socket;
    socket = INVALID_SOCKET_HANDLE;

    if (native_socket::closeSocket(socketToClose) < 0)
    {
        return native_socket::lastError();
    }

    return 0;
}

namespace
{

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

bool parseEndpoint(
    const NetworkEndpoint& endpoint,
    std::optional<uint32_t>& parsedAddress,
    bool allowDisabled
)
{
    const bool disabled = endpoint.address.empty() && endpoint.port == 0;
    if (disabled)
    {
        return allowDisabled;
    }

    if (endpoint.address.empty() || endpoint.port == 0)
    {
        return false;
    }

    parsedAddress = native_socket::parseIpv4(endpoint.address);
    return parsedAddress.has_value();
}

bool contains(
    const std::vector<SocketHandle>& sockets,
    SocketHandle socket
)
{
    return std::find(sockets.begin(), sockets.end(), socket) != sockets.end();
}

PollResult pollFailure(NetworkOperation operation, int errorCode)
{
    PollResult result;
    result.status = PollStatus::ERROR;
    result.failedOperation = operation;
    result.errorCode = errorCode;
    return result;
}

} // namespace
} // namespace comm
