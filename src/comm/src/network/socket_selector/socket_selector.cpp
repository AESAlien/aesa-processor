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
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("Socket wait requires a non-negative timeout");
    }
    if (_sockets.empty())
    {
        return {};
    }

    std::vector<SocketHandle> handles;
    handles.reserve(_sockets.size());
    for (const Socket* socket : _sockets)
    {
        const SocketHandle handle = socket->getHandle();
        handles.push_back(handle);
    }
    std::vector<SocketHandle> readyHandles;
    const char* operation = mode == WaitMode::READABLE ? "wait-readable" : "wait-writable";
    const int result = mode == WaitMode::READABLE
        ? native_socket::waitReadable(handles, timeout, readyHandles)
        : native_socket::waitWritable(handles, timeout, readyHandles);
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
