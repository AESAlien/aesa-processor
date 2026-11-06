#include <network/peer_connection/peer_connection.hpp>

#include <stdexcept>
#include <utility>

namespace comm
{

PeerConnection::PeerConnection(std::string name, CommEndpoint endpoint)
  : _name(std::move(name)),
    _endpoint(std::move(endpoint))
{
}

const std::string& PeerConnection::getName() const noexcept
{
    return _name;
}

bool PeerConnection::matches(const CommEndpoint& remoteEndpoint) const
{
    return _endpoint.getIntAddress() == remoteEndpoint.getIntAddress() &&
        _endpoint.getPort() == remoteEndpoint.getPort();
}

void PeerConnection::send(
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
) {
    if (data.empty() || timeout.count() < 0)
    {
        throw std::invalid_argument("Send requires data and a non-negative timeout");
    }

    const std::shared_ptr<Session> session = getSession();
    if (session)
    {
        session->send(data, timeout);
    }
}

void PeerConnection::attach(TcpSocket socket)
{
    auto session = std::make_shared<Session>(std::move(socket), _name);
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _session.swap(session);
    }
    // Release the previous session outside the lock.
}

std::shared_ptr<Session> PeerConnection::getSession() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _session;
}

void PeerConnection::detachIfCurrent(const std::shared_ptr<Session>& session)
{
    std::shared_ptr<Session> previousSession;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_session == session)
        {
            previousSession = std::move(_session);
        }
    }
}

} // namespace comm
