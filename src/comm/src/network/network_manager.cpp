#include <comm/network/network_manager.hpp>

#include <arpa/inet.h>
#include <cerrno>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace comm
{

namespace
{

constexpr int INVALID_SOCKET_FD = -1;
constexpr int RECEIVE_BUFFER_SIZE = 4096;

} // namespace

NetworkManager::NetworkManager(
    std::uint16_t port,
    const std::string& operatorConsoleIp,
    const std::string& scenarioSimulatorIp)
    : _port(port),
      _operatorConsoleIp(operatorConsoleIp),
      _scenarioSimulatorIp(scenarioSimulatorIp),
      _listenSocket(INVALID_SOCKET_FD),
      _operatorConsoleSocket(INVALID_SOCKET_FD),
      _scenarioSimulatorSocket(INVALID_SOCKET_FD)
{
}

NetworkManager::~NetworkManager()
{
    stop();
}

bool NetworkManager::start()
{
    if (_listenSocket != INVALID_SOCKET_FD)
    {
        return true;
    }

    _listenSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (_listenSocket == INVALID_SOCKET_FD)
    {
        return false;
    }

    int reuseAddress = 1;
    if (setsockopt(
            _listenSocket,
            SOL_SOCKET,
            SO_REUSEADDR,
            &reuseAddress,
            sizeof(reuseAddress)) < 0)
    {
        closeSocket(_listenSocket);
        return false;
    }

    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(_port);

    if (bind(
            _listenSocket,
            reinterpret_cast<sockaddr*>(&serverAddress),
            sizeof(serverAddress)) < 0)
    {
        closeSocket(_listenSocket);
        return false;
    }

    if (listen(_listenSocket, 2) < 0)
    {
        closeSocket(_listenSocket);
        return false;
    }

    return true;
}

bool NetworkManager::receive(
    ReceivedData& receivedData,
    int timeoutMilliseconds)
{
    if (_listenSocket == INVALID_SOCKET_FD || timeoutMilliseconds < 0)
    {
        return false;
    }

    fd_set readSockets;
    FD_ZERO(&readSockets);
    FD_SET(_listenSocket, &readSockets);

    int maximumSocket = _listenSocket;

    if (_operatorConsoleSocket != INVALID_SOCKET_FD)
    {
        FD_SET(_operatorConsoleSocket, &readSockets);
        if (_operatorConsoleSocket > maximumSocket)
        {
            maximumSocket = _operatorConsoleSocket;
        }
    }

    if (_scenarioSimulatorSocket != INVALID_SOCKET_FD)
    {
        FD_SET(_scenarioSimulatorSocket, &readSockets);
        if (_scenarioSimulatorSocket > maximumSocket)
        {
            maximumSocket = _scenarioSimulatorSocket;
        }
    }

    timeval timeout{};
    timeout.tv_sec = timeoutMilliseconds / 1000;
    timeout.tv_usec = (timeoutMilliseconds % 1000) * 1000;

    int selectedCount = select(
        maximumSocket + 1,
        &readSockets,
        nullptr,
        nullptr,
        &timeout);

    if (selectedCount <= 0)
    {
        return false;
    }

    if (FD_ISSET(_listenSocket, &readSockets))
    {
        acceptConnection();
    }

    if (_operatorConsoleSocket != INVALID_SOCKET_FD &&
        FD_ISSET(_operatorConsoleSocket, &readSockets))
    {
        return receiveFromSocket(
            _operatorConsoleSocket,
            NetworkPeer::OPERATOR_CONSOLE,
            receivedData);
    }

    if (_scenarioSimulatorSocket != INVALID_SOCKET_FD &&
        FD_ISSET(_scenarioSimulatorSocket, &readSockets))
    {
        return receiveFromSocket(
            _scenarioSimulatorSocket,
            NetworkPeer::SCENARIO_SIMULATOR,
            receivedData);
    }

    return false;
}

bool NetworkManager::send(
    NetworkPeer peer,
    const std::vector<std::uint8_t>& data)
{
    int socketFd = getSocket(peer);
    if (socketFd == INVALID_SOCKET_FD || data.empty())
    {
        return false;
    }

    if (!sendAll(socketFd, data))
    {
        disconnect(peer);
        return false;
    }

    return true;
}

bool NetworkManager::isConnected(NetworkPeer peer) const
{
    return getSocket(peer) != INVALID_SOCKET_FD;
}

void NetworkManager::disconnect(NetworkPeer peer)
{
    if (peer == NetworkPeer::OPERATOR_CONSOLE)
    {
        closeSocket(_operatorConsoleSocket);
        return;
    }

    closeSocket(_scenarioSimulatorSocket);
}

void NetworkManager::stop()
{
    closeSocket(_operatorConsoleSocket);
    closeSocket(_scenarioSimulatorSocket);
    closeSocket(_listenSocket);
}

bool NetworkManager::acceptConnection()
{
    sockaddr_in clientAddress{};
    socklen_t clientAddressSize = sizeof(clientAddress);

    int clientSocket = accept(
        _listenSocket,
        reinterpret_cast<sockaddr*>(&clientAddress),
        &clientAddressSize);

    if (clientSocket == INVALID_SOCKET_FD)
    {
        return false;
    }

    char clientIpBuffer[INET_ADDRSTRLEN]{};
    const char* clientIp = inet_ntop(
        AF_INET,
        &clientAddress.sin_addr,
        clientIpBuffer,
        sizeof(clientIpBuffer));

    if (clientIp == nullptr)
    {
        close(clientSocket);
        return false;
    }

    if (_operatorConsoleIp == clientIp)
    {
        closeSocket(_operatorConsoleSocket);
        setSocket(NetworkPeer::OPERATOR_CONSOLE, clientSocket);
        return true;
    }

    if (_scenarioSimulatorIp == clientIp)
    {
        closeSocket(_scenarioSimulatorSocket);
        setSocket(NetworkPeer::SCENARIO_SIMULATOR, clientSocket);
        return true;
    }

    close(clientSocket);
    return false;
}

bool NetworkManager::receiveFromSocket(
    int socketFd,
    NetworkPeer peer,
    ReceivedData& receivedData)
{
    std::uint8_t buffer[RECEIVE_BUFFER_SIZE];

    ssize_t receivedSize;
    do
    {
        receivedSize = recv(
            socketFd,
            buffer,
            sizeof(buffer),
            0);
    }
    while (receivedSize < 0 && errno == EINTR);

    if (receivedSize <= 0)
    {
        disconnect(peer);
        return false;
    }

    receivedData.peer = peer;
    receivedData.bytes.assign(buffer, buffer + receivedSize);
    return true;
}

bool NetworkManager::sendAll(
    int socketFd,
    const std::vector<std::uint8_t>& data)
{
    std::size_t totalSentSize = 0;

    while (totalSentSize < data.size())
    {
        ssize_t sentSize = ::send(
            socketFd,
            data.data() + totalSentSize,
            data.size() - totalSentSize,
            MSG_NOSIGNAL);

        if (sentSize < 0 && errno == EINTR)
        {
            continue;
        }

        if (sentSize <= 0)
        {
            return false;
        }

        totalSentSize += static_cast<std::size_t>(sentSize);
    }

    return true;
}

int NetworkManager::getSocket(NetworkPeer peer) const
{
    if (peer == NetworkPeer::OPERATOR_CONSOLE)
    {
        return _operatorConsoleSocket;
    }

    return _scenarioSimulatorSocket;
}

void NetworkManager::setSocket(NetworkPeer peer, int socketFd)
{
    if (peer == NetworkPeer::OPERATOR_CONSOLE)
    {
        _operatorConsoleSocket = socketFd;
        return;
    }

    _scenarioSimulatorSocket = socketFd;
}

void NetworkManager::closeSocket(int& socketFd)
{
    if (socketFd == INVALID_SOCKET_FD)
    {
        return;
    }

    shutdown(socketFd, SHUT_RDWR);
    close(socketFd);
    socketFd = INVALID_SOCKET_FD;
}

} // namespace comm
