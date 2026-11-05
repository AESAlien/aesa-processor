#include <network/socket_selector/socket_selector.hpp>

#include <network/socket_error/socket_error.hpp>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <winsock2.h>
#else
#include <sys/select.h>
#endif

#include <algorithm>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace comm
{

SocketSelector::SocketSelector(std::vector<const TcpSocket*> sockets)
    : _sockets(std::move(sockets))
{
#ifdef _WIN32
    if (_sockets.size() > FD_SETSIZE)
    {
        throw std::invalid_argument("Socket count exceeds the select limit");
    }
#endif
    for (const TcpSocket* socket : _sockets)
    {
        if (socket == nullptr)
        {
            throw std::invalid_argument("Socket wait requires non-null sockets");
        }
    }
}

SocketWaitStatus SocketSelector::waitReadable(
    std::chrono::microseconds timeout,
    std::vector<const TcpSocket*>& readySockets
) const {
    return wait(WaitMode::READABLE, timeout, readySockets);
}

SocketWaitStatus SocketSelector::waitWritable(
    std::chrono::microseconds timeout,
    std::vector<const TcpSocket*>& readySockets
) const {
    return wait(WaitMode::WRITABLE, timeout, readySockets);
}

SocketWaitStatus SocketSelector::wait(
    WaitMode mode,
    std::chrono::microseconds timeout,
    std::vector<const TcpSocket*>& readySockets
) const {
    readySockets.clear();
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("Socket wait requires a non-negative timeout");
    }
    if (_sockets.empty())
    {
        return SocketWaitStatus::TIMED_OUT;
    }

    fd_set watchedSockets;
    FD_ZERO(&watchedSockets);
    int descriptorLimit = 0;
    for (const TcpSocket* socket : _sockets)
    {
#ifndef _WIN32
        if (socket->_handle < 0 || socket->_handle >= FD_SETSIZE)
        {
            throw std::invalid_argument("Socket descriptor exceeds the select limit");
        }
        descriptorLimit = std::max(descriptorLimit, socket->_handle + 1);
#endif
        FD_SET(socket->_handle, &watchedSockets);
    }

    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timeout);
    const auto microseconds = timeout - seconds;
    timeval selectTimeout{};
    selectTimeout.tv_sec = static_cast<decltype(selectTimeout.tv_sec)>(seconds.count());
    selectTimeout.tv_usec = static_cast<decltype(selectTimeout.tv_usec)>(microseconds.count());

    fd_set* readSockets = mode == WaitMode::READABLE ? &watchedSockets : nullptr;
    fd_set* writeSockets = mode == WaitMode::WRITABLE ? &watchedSockets : nullptr;
    const char* operation = mode == WaitMode::READABLE ? "wait-readable" : "wait-writable";
    const int result = select(descriptorLimit, readSockets, writeSockets, nullptr, &selectTimeout);
    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        if (error.isInterrupted())
        {
            return SocketWaitStatus::INTERRUPTED;
        }
        throw std::system_error(error.code(), operation);
    }
    if (result == 0)
    {
        return SocketWaitStatus::TIMED_OUT;
    }

    for (const TcpSocket* socket : _sockets)
    {
        if (FD_ISSET(socket->_handle, &watchedSockets))
        {
            readySockets.push_back(socket);
        }
    }
    return SocketWaitStatus::READY;
}

} // namespace comm
