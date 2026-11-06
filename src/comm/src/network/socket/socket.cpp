#include <network/socket/socket.hpp>

#include <utility>

namespace comm
{

Socket::Socket() = default;

Socket::Socket(SocketResource socket)
  : _socket(std::move(socket))
{
}

Socket::~Socket() = default;

std::vector<SocketHandle> Socket::getHandles(const std::vector<const Socket*>& sockets)
{
    std::vector<SocketHandle> handles;
    handles.reserve(sockets.size());
    for (const Socket* socket : sockets)
    {
        handles.push_back(socket->_socket.getHandle());
    }
    return handles;
}

void Socket::close() noexcept
{
    _socket.close();
}

} // namespace comm
