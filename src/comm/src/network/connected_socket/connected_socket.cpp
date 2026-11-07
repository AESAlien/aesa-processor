#include <network/connected_socket/connected_socket.hpp>

#include <network/socket_error/socket_error.hpp>
#include <network/socket_selector/socket_selector.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace comm
{
namespace
{

int transferSize(std::size_t size) noexcept
{
    return static_cast<int>(std::min(
        size,
        static_cast<std::size_t>(std::numeric_limits<int>::max())
    ));
}

} // namespace

ConnectedSocket::ConnectedSocket(SocketHandle handle, uint32_t remoteAddress, uint16_t remotePort)
  : Socket(handle),
    _remoteAddress(remoteAddress),
    _remotePort(remotePort)
{
    if (native_socket::setNonBlocking(_handle) < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=set-non-blocking");
    }
}

CommEndpoint ConnectedSocket::getRemoteEndpoint() const
{
    const auto formattedAddress = native_socket::formatIpv4(_remoteAddress);

    if (!formattedAddress)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "operation=format-address");
    }
    return CommEndpoint(*formattedAddress, _remotePort);
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
    const int result = native_socket::sendBytes(_handle, pendingData, nativeSize);
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
    const int result = native_socket::receiveBytes(_handle, buffer.data(), nativeSize);
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
    native_socket::shutdownSocket(_handle);
}

} // namespace comm
