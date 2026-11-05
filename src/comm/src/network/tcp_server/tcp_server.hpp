#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/tcp_socket/tcp_socket.hpp>

#include <optional>

namespace comm
{

// Non-blocking IPv4 listen socket.
class TcpServer
{
public:
    // Throws ServerOpenError.
    explicit TcpServer(const CommEndpoint& endpoint);

    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;
    TcpServer(TcpServer&&) = delete;
    TcpServer& operator=(TcpServer&&) = delete;

    const TcpSocket& getSocket() const noexcept;

    // Returns a non-blocking socket the caller owns, or nullopt when no
    // connection is pending. Throws ConnectionError.
    std::optional<TcpSocket> accept();

private:
    TcpSocket _listenSocket;
};

} // namespace comm
