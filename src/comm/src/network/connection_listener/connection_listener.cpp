#include <network/connection_listener/connection_listener.hpp>

#include <comm/error/comm_error.hpp>

#include <algorithm>
#include <iostream>
#include <utility>

namespace comm
{

ConnectionListener::ConnectionListener(
    CommEndpoint endpoint,
    std::vector<std::reference_wrapper<PeerConnection>> connections
)
  : _endpoint(std::move(endpoint)),
    _connections(std::move(connections))
{
}

void ConnectionListener::start()
{
    if (!_listener)
    {
        _listener = std::make_unique<SessionListener>(_endpoint);
    }
}

std::optional<ReceivedFrames> ConnectionListener::poll(std::chrono::milliseconds timeout)
{
    if (!_listener)
    {
        throw ReceiveError("Communication data could not be received: server is not started");
    }
    const std::shared_ptr<Session> session = _listener->poll(timeout);
    if (!session)
    {
        return std::nullopt;
    }
    for (std::size_t offset = 0; offset < _connections.size(); ++offset)
    {
        const std::size_t index = (_nextConnection + offset) % _connections.size();
        PeerConnection& connection = _connections[index];
        auto frames = connection.receiveIfCurrent(session);
        if (frames)
        {
            _nextConnection = (index + 1) % _connections.size();
            if (frames->empty())
            {
                return std::nullopt;
            }
            return ReceivedFrames{connection.getName(), std::move(*frames)};
        }
    }

    for (PeerConnection& connection : _connections)
    {
        if (connection.matches(session->getRemoteEndpoint()))
        {
            connection.replaceSession(session);
            return std::nullopt;
        }
    }

    std::cerr << "[ConnectionListener] Rejected unknown connection: remoteAddress="
              << session->getRemoteEndpoint().getAddress()
              << ", remotePort=" << session->getRemoteEndpoint().getPort() << '\n';
    return std::nullopt;
}

} // namespace comm
