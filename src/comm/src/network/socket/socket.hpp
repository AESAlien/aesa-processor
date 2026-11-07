#pragma once

#include <network/native_socket/native_socket.hpp>

namespace comm
{

// Common owner for listening and connected sockets.
class Socket
{
public:
    virtual ~Socket();

    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

    // Borrows the handle; ownership remains with this socket.
    SocketHandle getHandle() const noexcept;
    void close() noexcept;

protected:
    Socket();
    // Takes ownership immediately, including on initialization failure.
    explicit Socket(SocketHandle handle);
    Socket(Socket&& other) noexcept;
    Socket& operator=(Socket&& other) noexcept;

    SocketHandle _handle = INVALID_SOCKET_HANDLE;

private:
    void initializeApi();
    void cleanupApi() noexcept;
    void validate() const;

    bool _apiInitialized = false;
};

} // namespace comm
