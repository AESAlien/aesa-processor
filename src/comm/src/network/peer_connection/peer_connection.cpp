#include <network/peer_connection/peer_connection.hpp>

#include <comm/error/comm_error.hpp>

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
    return _endpoint == remoteEndpoint;
}

void PeerConnection::send(
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
) {
    if (data.empty() || timeout.count() < 0)
    {
        throw std::invalid_argument("Send requires data and a non-negative timeout");
    }

    const std::shared_ptr<Session> session = snapshotSession();
    if (session)
    {
        try
        {
            session->send(data, timeout);
        }
        catch (const SendError& error)
        {
            throw SendError(std::string(error.what()) + ", peer=" + _name);
        }
    }
}

void PeerConnection::replaceSession(std::shared_ptr<Session> session)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _session.swap(session);
    }
    // Release the previous session outside the lock.
}

std::shared_ptr<Session> PeerConnection::snapshotSession() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _session;
}

bool PeerConnection::isConnected() const
{
    return static_cast<bool>(snapshotSession());
}

std::optional<std::vector<MessageFrame>> PeerConnection::receiveIfCurrent(
    const std::shared_ptr<Session>& session
)
{
    if (!session)
    {
        return std::nullopt;
    }
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_session != session)
        {
            return std::nullopt;
        }
    }

    try
    {
        auto frames = session->receive();
        if (!session->isOpen())
        {
            detachIfCurrent(session);
        }
        return std::move(frames);
    }
    catch (const ReceiveError& error)
    {
        if (!session->isOpen())
        {
            detachIfCurrent(session);
        }
        throw ReceiveError(std::string(error.what()) + ", peer=" + _name);
    }
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
