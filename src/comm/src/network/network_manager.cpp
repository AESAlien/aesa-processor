#include <comm/network/network_manager.hpp>

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace comm
{

namespace
{

const int INVALID_SOCKET_FD = -1;
const int RECEIVE_BUFFER_SIZE = 4096;

} // namespace

NetworkManager::NetworkManager(
    std::uint16_t port,
    const std::string& operatorConsoleIp,
    const std::string& scenarioSimulatorIp)
    : port_(port),
      operatorConsoleIp_(operatorConsoleIp),
      scenarioSimulatorIp_(scenarioSimulatorIp),
      listenSocket_(INVALID_SOCKET_FD),
      operatorConsoleSocket_(INVALID_SOCKET_FD),
      scenarioSimulatorSocket_(INVALID_SOCKET_FD)
{
}

NetworkManager::~NetworkManager()
{
    stop();
}

bool NetworkManager::start()
{
    if (listenSocket_ != INVALID_SOCKET_FD)
    {
        return true;
    }

    listenSocket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listenSocket_ == INVALID_SOCKET_FD)
    {
        return false;
    }

    int reuseAddress = 1;
    setsockopt(
        listenSocket_,
        SOL_SOCKET,
        SO_REUSEADDR,
        &reuseAddress,
        sizeof(reuseAddress));

    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(port_);

    if (bind(
            listenSocket_,
            reinterpret_cast<sockaddr*>(&serverAddress),
            sizeof(serverAddress)) < 0)
    {
        closeSocket(listenSocket_);
        return false;
    }

    if (listen(listenSocket_, 2) < 0)
    {
        closeSocket(listenSocket_);
        return false;
    }

    return true;
}

bool NetworkManager::receive(
    ReceivedData& receivedData,
    int timeoutMilliseconds)
{
    if (listenSocket_ == INVALID_SOCKET_FD)
    {
        return false;
    }

    fd_set readSockets;
    FD_ZERO(&readSockets);
    FD_SET(listenSocket_, &readSockets);

    int maximumSocket = listenSocket_;

    if (operatorConsoleSocket_ != INVALID_SOCKET_FD)
    {
        FD_SET(operatorConsoleSocket_, &readSockets);
        if (operatorConsoleSocket_ > maximumSocket)
        {
            maximumSocket = operatorConsoleSocket_;
        }
    }

    if (scenarioSimulatorSocket_ != INVALID_SOCKET_FD)
    {
        FD_SET(scenarioSimulatorSocket_, &readSockets);
        if (scenarioSimulatorSocket_ > maximumSocket)
        {
            maximumSocket = scenarioSimulatorSocket_;
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

    if (FD_ISSET(listenSocket_, &readSockets))
    {
        acceptConnection();
    }

    if (operatorConsoleSocket_ != INVALID_SOCKET_FD &&
        FD_ISSET(operatorConsoleSocket_, &readSockets))
    {
        return receiveFromSocket(
            operatorConsoleSocket_,
            NetworkPeer::OperatorConsole,
            receivedData);
    }

    if (scenarioSimulatorSocket_ != INVALID_SOCKET_FD &&
        FD_ISSET(scenarioSimulatorSocket_, &readSockets))
    {
        return receiveFromSocket(
            scenarioSimulatorSocket_,
            NetworkPeer::ScenarioSimulator,
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
    if (peer == NetworkPeer::OperatorConsole)
    {
        closeSocket(operatorConsoleSocket_);
        return;
    }

    closeSocket(scenarioSimulatorSocket_);
}

void NetworkManager::stop()
{
    closeSocket(operatorConsoleSocket_);
    closeSocket(scenarioSimulatorSocket_);
    closeSocket(listenSocket_);
}

bool NetworkManager::acceptConnection()
{
    sockaddr_in clientAddress{};
    socklen_t clientAddressSize = sizeof(clientAddress);

    int clientSocket = accept(
        listenSocket_,
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

    if (operatorConsoleIp_ == clientIp)
    {
        closeSocket(operatorConsoleSocket_);
        setSocket(NetworkPeer::OperatorConsole, clientSocket);
        return true;
    }

    if (scenarioSimulatorIp_ == clientIp)
    {
        closeSocket(scenarioSimulatorSocket_);
        setSocket(NetworkPeer::ScenarioSimulator, clientSocket);
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

    ssize_t receivedSize = recv(
        socketFd,
        buffer,
        sizeof(buffer),
        0);

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
            0);

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
    if (peer == NetworkPeer::OperatorConsole)
    {
        return operatorConsoleSocket_;
    }

    return scenarioSimulatorSocket_;
}

void NetworkManager::setSocket(NetworkPeer peer, int socketFd)
{
    if (peer == NetworkPeer::OperatorConsole)
    {
        operatorConsoleSocket_ = socketFd;
        return;
    }

    scenarioSimulatorSocket_ = socketFd;
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