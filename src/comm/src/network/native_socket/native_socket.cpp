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
#include <stdexcept>

namespace comm::native_socket
{
namespace
{

#ifdef _WIN32
using SocketLength = int;
#else
using SocketLength = socklen_t;
#endif

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

int cleanup() noexcept
{
#ifdef _WIN32
    if (WSACleanup() == SOCKET_ERROR)
    {
        return WSAGetLastError();
    }
#endif
    return 0;
}

int defaultBacklog() noexcept
{
    return SOMAXCONN;
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

SocketHandle acceptSocket(SocketHandle socket)
{
    return static_cast<SocketHandle>(::accept(socket, nullptr, nullptr));
}

int remoteEndpoint(SocketHandle socket, uint32_t& address, uint16_t& port)
{
    sockaddr_in remote{};
    SocketLength length = sizeof(remote);
    const int result = getpeername(socket, reinterpret_cast<sockaddr*>(&remote), &length);
    if (result == 0)
    {
        address = remote.sin_addr.s_addr;
        port = ntohs(remote.sin_port);
    }
    return result;
}

void validateRegistrationCount(std::size_t count)
{
#ifdef _WIN32
    if (count > FD_SETSIZE)
    {
        throw std::invalid_argument("SocketResource count exceeds the select limit");
    }
#else
    static_cast<void>(count);
#endif
}

int wait(
    const std::vector<SocketHandle>& sockets,
    WaitMode mode,
    std::chrono::microseconds timeout,
    std::vector<SocketHandle>& readySockets
) {
    readySockets.clear();
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("SocketResource wait requires a non-negative timeout");
    }
    validateRegistrationCount(sockets.size());
    if (sockets.empty())
    {
        return 0;
    }
    fd_set watched;
    FD_ZERO(&watched);
    int descriptorLimit = 0;
    for (SocketHandle socket : sockets)
    {
#ifdef _WIN32
        if (socket == INVALID_SOCKET_HANDLE)
#else
        if (socket < 0 || socket >= FD_SETSIZE)
#endif
        {
            throw std::invalid_argument("SocketResource descriptor exceeds the select limit");
        }
#ifndef _WIN32
        descriptorLimit = std::max(descriptorLimit, socket + 1);
#endif
        FD_SET(socket, &watched);
    }
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timeout);
    const auto microseconds = timeout - seconds;
    timeval selectTimeout{};
    selectTimeout.tv_sec = static_cast<decltype(selectTimeout.tv_sec)>(seconds.count());
    selectTimeout.tv_usec = static_cast<decltype(selectTimeout.tv_usec)>(microseconds.count());
    fd_set* readSockets = mode == WaitMode::READABLE ? &watched : nullptr;
    fd_set* writeSockets = mode == WaitMode::WRITABLE ? &watched : nullptr;
    const int result = select(descriptorLimit, readSockets, writeSockets, nullptr, &selectTimeout);
    if (result > 0)
    {
        for (SocketHandle socket : sockets)
        {
            if (FD_ISSET(socket, &watched))
            {
                readySockets.push_back(socket);
            }
        }
    }
    return result;
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

int shutdownSocket(SocketHandle socket)
{
#ifdef _WIN32
    return shutdown(socket, SD_BOTH);
#else
    return shutdown(socket, SHUT_RDWR);
#endif
}

int closeSocket(SocketHandle socket) noexcept
{
    shutdownSocket(socket);
#ifdef _WIN32
    return closesocket(socket);
#else
    return close(socket);
#endif
}

} // namespace comm::native_socket
