#pragma once

#include <network/socket_resource/socket_resource.hpp>

#include <vector>

namespace comm
{

// Common owner for listening and connected sockets.
class Socket
{
public:
    virtual ~Socket();

    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

    void close() noexcept;

    // Returns native handles for the supplied sockets.
    static std::vector<SocketHandle> getHandles(const std::vector<const Socket*>& sockets);

protected:
    Socket();
    explicit Socket(SocketResource socket);
    Socket(Socket&&) noexcept = default;
    Socket& operator=(Socket&&) noexcept = default;

    SocketResource _socket;
};

} // namespace comm
