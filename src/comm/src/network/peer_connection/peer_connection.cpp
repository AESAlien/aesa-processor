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

bool PeerConnection::matches(const Session& session) const
{
    return matches(session.getRemoteEndpoint());
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

std::vector<MessageFrame> PeerConnection::receive()
{
    const std::shared_ptr<Session> session = getSession();
    if (!session)
    {
        return {};
    }

    try
    {
        std::vector<MessageFrame> frames = session->receive();
        if (!session->isOpen())
        {
            setSession(nullptr);
        }
        return frames;
    }
    catch (const ReceiveError& error)
    {
        if (!session->isOpen())
        {
            setSession(nullptr);
        }
        throw ReceiveError(std::string(error.what()) + ", peer=" + _name);
    }
}

std::shared_ptr<Session> PeerConnection::getSession() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _session;
}

void PeerConnection::setSession(std::shared_ptr<Session> session)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _session.swap(session);
    }
    // The previous session, now in session, is released here outside the lock.
}

} // namespace comm
