#include <network/socket_selector/socket_selector.hpp>

#include <network/socket_error/socket_error.hpp>
#include <algorithm>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace comm
{

SocketSelector::SocketSelector(std::vector<const Socket*> sockets)
  : _sockets(std::move(sockets))
{
    native_socket::validateRegistrationCount(_sockets.size());
    for (const Socket* socket : _sockets)
    {
        if (socket == nullptr)
        {
            throw std::invalid_argument("SocketSelector requires non-null sockets");
        }
    }
}

std::vector<const Socket*> SocketSelector::waitReadable(
    std::chrono::microseconds timeout
) const {
    return wait(WaitMode::READABLE, timeout);
}

std::vector<const Socket*> SocketSelector::waitWritable(
    std::chrono::microseconds timeout
) const {
    return wait(WaitMode::WRITABLE, timeout);
}

std::vector<const Socket*> SocketSelector::wait(
    WaitMode mode,
    std::chrono::microseconds timeout
) const {
    const std::vector<SocketHandle> handles = Socket::getHandles(_sockets);
    std::vector<SocketHandle> readyHandles;
    const auto nativeMode = mode == WaitMode::READABLE
        ? native_socket::WaitMode::READABLE
        : native_socket::WaitMode::WRITABLE;
    const char* operation = mode == WaitMode::READABLE ? "wait-readable" : "wait-writable";
    const int result = native_socket::wait(handles, nativeMode, timeout, readyHandles);
    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        if (error.isInterrupted())
        {
            return {};
        }
        throw std::system_error(error.code(), operation);
    }
    if (result == 0)
    {
        return {};
    }

    std::vector<const Socket*> readySockets;
    for (std::size_t index = 0; index < _sockets.size(); ++index)
    {
        if (std::find(readyHandles.begin(), readyHandles.end(), handles[index]) != readyHandles.end())
        {
            readySockets.push_back(_sockets[index]);
        }
    }
    return readySockets;
}

} // namespace comm
