#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/listen_socket/listen_socket.hpp>
#include <network/session/session.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

namespace comm
{

enum class ServerEventKind
{
    NONE,
    NEW_SESSION,
    READABLE
};

struct ServerEvent
{
    ServerEventKind kind;
    std::shared_ptr<Session> session;
};

// Accepts connections and waits on the listener together with the given
// sessions. It keeps no sessions itself, so only the sessions passed to each
// poll() are watched and the caller decides which ones are current.
class TcpServer
{
public:
    // Throws ServerOpenError.
    explicit TcpServer(const CommEndpoint& endpoint);

    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;
    TcpServer(TcpServer&&) = delete;
    TcpServer& operator=(TcpServer&&) = delete;

    // NEW_SESSION returns a newly accepted session and READABLE returns one of
    // the given sessions. Null sessions are ignored. Timeout and interruption
    // return NONE. Sockets that are ready together are returned in turn.
    // Throws ConnectionError if accepting fails, ReceiveError if waiting fails.
    ServerEvent poll(
        const std::vector<std::shared_ptr<Session>>& sessions,
        std::chrono::milliseconds timeout
    );

private:
    ServerEvent accept();

    ListenSocket _listenSocket;
    std::size_t _nextReadyIndex = 0;
};

} // namespace comm
