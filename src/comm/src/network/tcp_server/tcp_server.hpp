#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/native_socket/native_socket.hpp>

#include <cstdint>
#include <optional>

namespace comm
{

struct AcceptedSocket
{
    SocketHandle socket;
    uint32_t address;
    uint16_t port;
};

// Non-blocking IPv4 listen socket. Initializes the socket API while alive.
class TcpServer
{
public:
    // Throws ServerOpenError.
    explicit TcpServer(const CommEndpoint& endpoint);
    ~TcpServer();

    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;
    TcpServer(TcpServer&&) = delete;
    TcpServer& operator=(TcpServer&&) = delete;

    SocketHandle getSocket() const noexcept;

    // Returns a non-blocking socket the caller owns, or nullopt when no
    // connection is pending. Throws ConnectionError.
    std::optional<AcceptedSocket> accept();

private:
    [[noreturn]] void failOpen(const char* operation);

    SocketHandle _listenSocket = INVALID_SOCKET_HANDLE;
};

} // namespace comm
