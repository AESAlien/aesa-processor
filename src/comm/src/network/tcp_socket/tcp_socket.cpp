#include <network/tcp_socket/tcp_socket.hpp>

#include <network/socket_error/socket_error.hpp>
#include <network/socket_selector/socket_selector.hpp>

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
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace comm
{
namespace
{

#ifdef _WIN32
using SocketLength = int;
#else
using SocketLength = socklen_t;
#endif

int transferSize(std::size_t size) noexcept
{
    return static_cast<int>(std::min(
        size,
        static_cast<std::size_t>(std::numeric_limits<int>::max())));
}

} // namespace

const int TcpSocket::DEFAULT_BACKLOG = SOMAXCONN;

TcpSocket::TcpSocket(int backlog)
    : _backlog(backlog)
{
    try
    {
        initializeApi();
        _handle = static_cast<SocketHandle>(::socket(AF_INET, SOCK_STREAM, 0));
        validate();
    }
    catch (...)
    {
        close();
        cleanupApi();
        throw;
    }
}

TcpSocket::TcpSocket(SocketHandle handle, AdoptHandle)
    : _handle(handle)
{
    try
    {
        initializeApi();
        validate();
    }
    catch (...)
    {
        close();
        cleanupApi();
        throw;
    }
}

TcpSocket::~TcpSocket()
{
    close();
    cleanupApi();
}

TcpSocket::TcpSocket(TcpSocket&& other) noexcept
    : _handle(std::exchange(other._handle, INVALID_SOCKET_HANDLE)),
      _backlog(other._backlog)
{
#ifdef _WIN32
    _apiInitialized = std::exchange(other._apiInitialized, false);
#endif
}

TcpSocket& TcpSocket::operator=(TcpSocket&& other) noexcept
{
    if (this != &other)
    {
        close();
        cleanupApi();
        _handle = std::exchange(other._handle, INVALID_SOCKET_HANDLE);
        _backlog = other._backlog;
#ifdef _WIN32
        _apiInitialized = std::exchange(other._apiInitialized, false);
#endif
    }
    return *this;
}

void TcpSocket::initializeApi()
{
#ifdef _WIN32
    WSADATA socketApiData{};
    const int errorCode = WSAStartup(MAKEWORD(2, 2), &socketApiData);
    if (errorCode != 0)
    {
        const SocketError error(errorCode);
        throw std::system_error(error.code(), "socket-api-startup");
    }
    _apiInitialized = true;
#endif
}

void TcpSocket::cleanupApi() noexcept
{
#ifdef _WIN32
    if (std::exchange(_apiInitialized, false))
    {
        WSACleanup();
    }
#endif
}

void TcpSocket::validate() const
{
    if (_handle == INVALID_SOCKET_HANDLE)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "create-socket");
    }
}

void TcpSocket::setReuseAddress()
{
    int reuseAddress = 1;
#ifdef _WIN32
    const char* optionValue = reinterpret_cast<const char*>(&reuseAddress);
#else
    const int* optionValue = &reuseAddress;
#endif

    const int result = setsockopt(
        _handle,
        SOL_SOCKET,
        SO_REUSEADDR,
        optionValue,
        sizeof(reuseAddress)
    );

    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=set-reuse-address");
    }
}

void TcpSocket::setNonBlocking()
{
#ifdef _WIN32
    u_long enabled = 1;
    const int result = ioctlsocket(_handle, FIONBIO, &enabled);
#else
    int flags = fcntl(_handle, F_GETFL, 0);
    if (flags < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=get-socket-flags");
    }

    const int result = fcntl(_handle, F_SETFL, flags | O_NONBLOCK);
#endif
    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=set-non-blocking");
    }
}

void TcpSocket::bind(const CommEndpoint& endpoint)
{
    sockaddr_in socketAddress{};
    socketAddress.sin_family = AF_INET;
    socketAddress.sin_addr.s_addr = endpoint.getIntAddress();
    socketAddress.sin_port = htons(endpoint.getPort());

    int result = ::bind(
        _handle,
        reinterpret_cast<sockaddr*>(&socketAddress),
        sizeof(socketAddress)
    );

    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=bind");
    }
}

void TcpSocket::listen()
{
    int result = ::listen(_handle, _backlog);

    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=listen");
    }
}

std::optional<TcpSocket> TcpSocket::accept()
{
    const SocketHandle clientHandle = static_cast<SocketHandle>(::accept(
        _handle,
        nullptr,
        nullptr)
    );

    if (clientHandle == INVALID_SOCKET_HANDLE)
    {
        const SocketError error = SocketError::captureLast();
        if (error.isWouldBlock() || error.isInterrupted())
        {
            return std::nullopt;
        }
        throw std::system_error(error.code(), "accept");
    }

    return TcpSocket(clientHandle, AdoptHandle{});
}

CommEndpoint TcpSocket::getRemoteEndpoint() const
{
    sockaddr_in clientAddress{};
    SocketLength clientAddressSize = sizeof(clientAddress);

    const int result = getpeername(
        _handle,
        reinterpret_cast<sockaddr*>(&clientAddress),
        &clientAddressSize
    );
    
    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=get-remote-endpoint");
    }

    char address[INET_ADDRSTRLEN]{};
    const char* formattedAddress = inet_ntop(
        AF_INET,
        &clientAddress.sin_addr,
        address,
        sizeof(address)
    );
    
    if (formattedAddress == nullptr)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=format-address");
    }
    return CommEndpoint(address, ntohs(clientAddress.sin_port));
}

void TcpSocket::waitWritable(std::chrono::microseconds timeout) const
{
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("Write wait requires a non-negative timeout");
    }
    using Clock = std::chrono::steady_clock;
    waitWritableUntil(Clock::now() + timeout);
}

void TcpSocket::waitWritableUntil(std::chrono::steady_clock::time_point deadline) const
{
    using Clock = std::chrono::steady_clock;
    const SocketSelector selector({this});
    std::vector<const TcpSocket*> readySockets;
    for (;;)
    {
        const Clock::time_point now = Clock::now();
        const std::chrono::microseconds remaining = now < deadline
            ? std::chrono::duration_cast<std::chrono::microseconds>(deadline - now)
            : std::chrono::microseconds{0};
        const SocketWaitStatus status = selector.waitWritable(remaining, readySockets);
        if (status == SocketWaitStatus::READY)
        {
            return;
        }
        if (status == SocketWaitStatus::TIMED_OUT || Clock::now() >= deadline)
        {
            throw std::system_error(std::make_error_code(std::errc::timed_out), "wait-writable");
        }
    }
}

std::optional<std::size_t> TcpSocket::sendBytes(
    const std::vector<uint8_t>& data,
    std::size_t offset
) {
    if (offset > data.size())
    {
        throw std::out_of_range("Send offset exceeds the buffer size");
    }
    if (offset == data.size())
    {
        return 0;
    }
    const int nativeSize = transferSize(data.size() - offset);
    const uint8_t* pendingData = data.data() + offset;
#ifdef _WIN32
    const char* nativeData = reinterpret_cast<const char*>(pendingData);
    const int result = ::send(_handle, nativeData, nativeSize, 0);
#else
    const int result = static_cast<int>(::send(_handle, pendingData, nativeSize, MSG_NOSIGNAL));
#endif
    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        if (error.isWouldBlock() || error.isInterrupted())
        {
            return std::nullopt;
        }
        throw std::system_error(error.code(), "send");
    }
    return static_cast<std::size_t>(result);
}

std::optional<std::vector<uint8_t>> TcpSocket::receiveBytes(std::size_t maximumBytes)
{
    if (maximumBytes == 0)
    {
        throw std::invalid_argument("Receive requires a positive maximum byte count");
    }
    const int nativeSize = transferSize(maximumBytes);
    std::vector<uint8_t> buffer(static_cast<std::size_t>(nativeSize));
#ifdef _WIN32
    char* nativeBuffer = reinterpret_cast<char*>(buffer.data());
#else
    uint8_t* nativeBuffer = buffer.data();
#endif
    const int result = static_cast<int>(recv(_handle, nativeBuffer, nativeSize, 0));
    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        if (error.isWouldBlock() || error.isInterrupted())
        {
            return std::nullopt;
        }
        throw std::system_error(error.code(), "receive");
    }
    buffer.resize(static_cast<std::size_t>(result));
    return buffer;
}

void TcpSocket::shutdown()
{
#ifdef _WIN32
    const int result = ::shutdown(_handle, SD_BOTH);
#else
    const int result = ::shutdown(_handle, SHUT_RDWR);
#endif

    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=shutdown");
    }
}

void TcpSocket::close() noexcept
{
    if (_handle == INVALID_SOCKET_HANDLE)
    {
        return;
    }

    const SocketHandle handle = std::exchange(_handle, INVALID_SOCKET_HANDLE);
#ifdef _WIN32
    ::shutdown(handle, SD_BOTH);
    closesocket(handle);
#else
    ::shutdown(handle, SHUT_RDWR);
    ::close(handle);
#endif
}

} // namespace comm
