#pragma once

#include <network/native_socket/native_socket.hpp>

namespace comm
{

// Internal move-only owner of a native handle and one socket API startup.
// No protocol operations are exposed. Do not move or close during an operation.
class SocketResource
{
public:
    SocketResource();
    ~SocketResource();
    SocketResource(const SocketResource&) = delete;
    SocketResource& operator=(const SocketResource&) = delete;
    SocketResource(SocketResource&& other) noexcept;
    SocketResource& operator=(SocketResource&& other) noexcept;
    SocketHandle getHandle() const noexcept;
    void close() noexcept;

private:
    friend class ListenSocket;
    friend class ConnectedSocket;
    // Takes ownership immediately, including on initialization failure.
    explicit SocketResource(SocketHandle handle);
    void initializeApi();
    void cleanupApi() noexcept;
    void validate() const;

    SocketHandle _handle = INVALID_SOCKET_HANDLE;
    bool _apiInitialized = false;
};

} // namespace comm
