#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/connected_socket/connected_socket.hpp>
#include <network/socket/socket.hpp>

#include <optional>

namespace comm
{

// Bound, listening, non-blocking IPv4 TCP socket. No data I/O operations.
// After move/close only move assignment, close and destruction are supported.
class ListenSocket : public Socket
{
public:
    // Throws ServerOpenError. Construction completes all listener setup.
    explicit ListenSocket(
        const CommEndpoint& endpoint,
        int backlog = native_socket::defaultBacklog()
    );
    ListenSocket(const ListenSocket&) = delete;
    ListenSocket& operator=(const ListenSocket&) = delete;
    ListenSocket(ListenSocket&&) noexcept = default;
    ListenSocket& operator=(ListenSocket&&) noexcept = default;

    // Returns nullopt on would-block/interruption, otherwise an independently
    // owned non-blocking connection. Native failures throw ConnectionError.
    std::optional<ConnectedSocket> accept();
};

} // namespace comm
