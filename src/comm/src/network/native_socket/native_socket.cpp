#include <network/native_socket/native_socket.hpp>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <algorithm>

namespace comm::native_socket
{
namespace
{

#ifdef _WIN32
using SocketLength = int;
#else
using SocketLength = socklen_t;
#endif

timeval makeTimeval(std::chrono::microseconds timeout);

int selectSockets(
    SocketHandle maximumSocket,
    fd_set* readSockets,
    fd_set* writeSockets,
    std::chrono::microseconds timeout
);

} // namespace

int startup()
{
#ifdef _WIN32
    WSADATA socketApiData{};
    return WSAStartup(MAKEWORD(2, 2), &socketApiData);
#else
    return 0;
#endif
}

int cleanup()
{
#ifdef _WIN32
    if (WSACleanup() == SOCKET_ERROR)
    {
        return lastError();
    }
#endif
    return 0;
}

int lastError()
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

std::optional<uint32_t> parseIpv4(const std::string& address)
{
    in_addr parsedAddress{};
    if (inet_pton(AF_INET, address.c_str(), &parsedAddress) != 1)
    {
        return std::nullopt;
    }

    return parsedAddress.s_addr;
}

std::optional<std::string> formatIpv4(uint32_t address)
{
    in_addr nativeAddress{};
    nativeAddress.s_addr = address;

    char addressBuffer[INET_ADDRSTRLEN]{};
    if (inet_ntop(
            AF_INET,
            &nativeAddress,
            addressBuffer,
            sizeof(addressBuffer)) == nullptr)
    {
        return std::nullopt;
    }

    return std::string(addressBuffer);
}

SocketHandle createTcpSocket()
{
    return static_cast<SocketHandle>(socket(AF_INET, SOCK_STREAM, 0));
}

int setReuseAddress(SocketHandle socket)
{
    int reuseAddress = 1;
    return setsockopt(
        socket,
        SOL_SOCKET,
        SO_REUSEADDR,
#ifdef _WIN32
        reinterpret_cast<const char*>(&reuseAddress),
#else
        &reuseAddress,
#endif
        sizeof(reuseAddress));
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

int bindSocket(SocketHandle socket, uint32_t address, uint16_t port)
{
    sockaddr_in socketAddress{};
    socketAddress.sin_family = AF_INET;
    socketAddress.sin_addr.s_addr = address;
    socketAddress.sin_port = htons(port);

    return bind(
        socket,
        reinterpret_cast<sockaddr*>(&socketAddress),
        sizeof(socketAddress));
}

int listenSocket(SocketHandle socket, int backlog)
{
    return listen(socket, backlog);
}

SocketHandle acceptSocket(
    SocketHandle listenSocket,
    uint32_t& remoteAddress,
    uint16_t& remotePort
)
{
    sockaddr_in clientAddress{};
    SocketLength clientAddressSize = sizeof(clientAddress);

    const SocketHandle clientSocket = static_cast<SocketHandle>(accept(
        listenSocket,
        reinterpret_cast<sockaddr*>(&clientAddress),
        &clientAddressSize));

    remoteAddress = clientAddress.sin_addr.s_addr;
    remotePort = ntohs(clientAddress.sin_port);
    return clientSocket;
}

int waitReadable(
    const std::vector<SocketHandle>& sockets,
    std::chrono::microseconds timeout,
    std::vector<SocketHandle>& readySockets
)
{
    readySockets.clear();

    fd_set readSockets;
    FD_ZERO(&readSockets);
    for (SocketHandle socket : sockets)
    {
        FD_SET(socket, &readSockets);
    }

    const SocketHandle maximumSocket =
        *std::max_element(sockets.begin(), sockets.end());

    const int selectedCount = selectSockets(
        maximumSocket,
        &readSockets,
        nullptr,
        timeout);

    if (selectedCount > 0)
    {
        for (SocketHandle socket : sockets)
        {
            if (FD_ISSET(socket, &readSockets))
            {
                readySockets.push_back(socket);
            }
        }
    }

    return selectedCount;
}

int waitWritable(
    SocketHandle socket,
    std::chrono::microseconds timeout
)
{
    fd_set writeSockets;
    FD_ZERO(&writeSockets);
    FD_SET(socket, &writeSockets);

    return selectSockets(
        socket,
        nullptr,
        &writeSockets,
        timeout);
}

int sendBytes(SocketHandle socket, const uint8_t* data, int size)
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

int receiveBytes(SocketHandle socket, uint8_t* buffer, int size)
{
#ifdef _WIN32
    return recv(socket, reinterpret_cast<char*>(buffer), size, 0);
#else
    return static_cast<int>(recv(socket, buffer, size, 0));
#endif
}

void shutdownSocket(SocketHandle socket)
{
#ifdef _WIN32
    shutdown(socket, SD_BOTH);
#else
    shutdown(socket, SHUT_RDWR);
#endif
}

int closeSocket(SocketHandle socket)
{
    shutdownSocket(socket);
#ifdef _WIN32
    return closesocket(socket);
#else
    return close(socket);
#endif
}

namespace
{

timeval makeTimeval(std::chrono::microseconds timeout)
{
    using Seconds = std::chrono::seconds;

    const auto seconds = std::chrono::duration_cast<Seconds>(timeout);
    const auto microseconds = timeout - seconds;

    timeval value{};
    value.tv_sec = static_cast<decltype(value.tv_sec)>(seconds.count());
    value.tv_usec = static_cast<decltype(value.tv_usec)>(microseconds.count());
    return value;
}

int selectSockets(
    SocketHandle maximumSocket,
    fd_set* readSockets,
    fd_set* writeSockets,
    std::chrono::microseconds timeout
)
{
    timeval selectTimeout = makeTimeval(timeout);

#ifdef _WIN32
    static_cast<void>(maximumSocket);
    return select(0, readSockets, writeSockets, nullptr, &selectTimeout);
#else
    return select(
        maximumSocket + 1,
        readSockets,
        writeSockets,
        nullptr,
        &selectTimeout);
#endif
}

} // namespace
} // namespace comm::native_socket
