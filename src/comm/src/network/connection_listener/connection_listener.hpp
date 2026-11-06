#pragma once

#include <network/peer_connection/peer_connection.hpp>
#include <network/network_manager/received_frames.hpp>
#include <network/session_listener/session_listener.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace comm
{

// Borrows configured peers. start() and poll() run on the receiving thread;
// peers must outlive this listener. Sending may run concurrently.
class ConnectionListener
{
public:
    ConnectionListener(
        CommEndpoint endpoint,
        std::vector<std::reference_wrapper<PeerConnection>> connections
    );
    void start();
    std::optional<ReceivedFrames> poll(std::chrono::milliseconds timeout);

private:
    CommEndpoint _endpoint;
    std::vector<std::reference_wrapper<PeerConnection>> _connections;
    std::unique_ptr<SessionListener> _listener;
    std::size_t _nextConnection = 0;
};

} // namespace comm
