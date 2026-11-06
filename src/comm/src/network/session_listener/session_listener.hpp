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

// Session-layer adapter for accepting and monitoring TCP connections.
class SessionListener
{
public:
    explicit SessionListener(const CommEndpoint& endpoint);
    std::shared_ptr<Session> accept();

    // Returns a newly accepted or readable session; timeout/interruption returns nullptr.
    std::shared_ptr<Session> poll(std::chrono::milliseconds timeout);

private:
    ListenSocket _listenSocket;
    std::vector<std::weak_ptr<Session>> _sessions;
    std::size_t _nextReadyIndex = 0;
};

} // namespace comm
