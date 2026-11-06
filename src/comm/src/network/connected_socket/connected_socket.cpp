#include <network/connected_socket/connected_socket.hpp>

#include <network/socket_error/socket_error.hpp>
#include <network/socket_selector/socket_selector.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace comm
{
namespace
{

int transferSize(std::size_t size) noexcept
{
    return static_cast<int>(std::min(
        size,
        static_cast<std::size_t>(std::numeric_limits<int>::max())));
}

} // namespace

ConnectedSocket::ConnectedSocket(SocketResource socket)
    : Socket(std::move(socket))
{
    if (native_socket::setNonBlocking(_socket._handle) < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=set-non-blocking");
    }
}

CommEndpoint ConnectedSocket::getRemoteEndpoint() const
{
    uint32_t address = 0;
    uint16_t port = 0;
    const int result = native_socket::remoteEndpoint(_socket._handle, address, port);

    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=get-remote-endpoint");
    }

    const auto formattedAddress = native_socket::formatIpv4(address);

    if (!formattedAddress)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=format-address");
    }
    return CommEndpoint(*formattedAddress, port);
}

void ConnectedSocket::waitWritable(std::chrono::microseconds timeout) const
{
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("Write wait requires a non-negative timeout");
    }
    using Clock = std::chrono::steady_clock;
    waitWritableUntil(Clock::now() + timeout);
}

void ConnectedSocket::waitWritableUntil(std::chrono::steady_clock::time_point deadline) const
{
    using Clock = std::chrono::steady_clock;
    const SocketSelector selector({this});
    for (;;)
    {
        const Clock::time_point now = Clock::now();
        const std::chrono::microseconds remaining = now < deadline
            ? std::chrono::duration_cast<std::chrono::microseconds>(deadline - now)
            : std::chrono::microseconds{0};
        if (!selector.waitWritable(remaining).empty())
        {
            return;
        }
        if (Clock::now() >= deadline)
        {
            throw std::system_error(std::make_error_code(std::errc::timed_out), "wait-writable");
        }
    }
}

std::optional<std::size_t> ConnectedSocket::sendBytes(
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
    const int result = native_socket::sendBytes(_socket._handle, pendingData, nativeSize);
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

std::optional<std::vector<uint8_t>> ConnectedSocket::receiveBytes(std::size_t maximumBytes)
{
    if (maximumBytes == 0)
    {
        throw std::invalid_argument("Receive requires a positive maximum byte count");
    }
    const int nativeSize = transferSize(maximumBytes);
    std::vector<uint8_t> buffer(static_cast<std::size_t>(nativeSize));
    const int result = native_socket::receiveBytes(_socket._handle, buffer.data(), nativeSize);
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

void ConnectedSocket::shutdown()
{
    const int result = native_socket::shutdownSocket(_socket._handle);

    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=shutdown");
    }
}

} // namespace comm
