#include <network/network_manager/network_manager.hpp>

#include <comm/error/comm_error.hpp>
#include <network/network_error.hpp>
#include <network/socket_selector/socket_selector.hpp>

#include <algorithm>
#include <iostream>
#include <system_error>
#include <utility>

namespace comm
{
namespace
{

constexpr char RECEIVE_FAILURE[] = "Communication data could not be received";

bool contains(
    const std::vector<const TcpSocket*>& sockets,
    const TcpSocket& socket
);

CommEndpoint readRemoteEndpoint(const TcpSocket& socket);
void reportRejectedConnection(const CommEndpoint& remoteEndpoint);

} // namespace

NetworkManager::NetworkManager(
    const CommEndpoint& serverEndpoint,
    std::vector<std::reference_wrapper<PeerConnection>> connections
) : _serverEndpoint(serverEndpoint),
    _connections(std::move(connections))
{
}

void NetworkManager::start()
{
    if (_server)
    {
        return;
    }

    _server = std::make_unique<TcpServer>(_serverEndpoint);
}

std::optional<ReceivedFrames> NetworkManager::poll(
    std::chrono::milliseconds timeout
) {
    if (!_server)
    {
        throw ReceiveError(
            std::string(RECEIVE_FAILURE) + ": server is not started");
    }

    // Keep each registered socket alive until this poll finishes.
    std::vector<std::shared_ptr<Session>> sessions;
    std::vector<const TcpSocket*> sockets{&_server->getSocket()};
    for (const PeerConnection& connection : _connections)
    {
        sessions.push_back(connection.getSession());
        if (sessions.back())
        {
            sockets.push_back(&sessions.back()->getSocket());
        }
    }

    std::vector<const TcpSocket*> readySockets;
    SocketWaitStatus status;
    try
    {
        const SocketSelector selector(std::move(sockets));
        status = selector.waitReadable(timeout, readySockets);
    }
    catch (const std::system_error& error)
    {
        throwSystemError<ReceiveError>(RECEIVE_FAILURE, "wait-readable", error.code().value());
    }
    if (status != SocketWaitStatus::READY)
    {
        return std::nullopt;
    }

    if (contains(readySockets, _server->getSocket()))
    {
        acceptSession();
        return std::nullopt;
    }

    for (std::size_t offset = 0; offset < _connections.size(); ++offset)
    {
        const std::size_t index = (_nextConnection + offset) % _connections.size();
        const std::shared_ptr<Session>& session = sessions[index];
        if (session && contains(readySockets, session->getSocket()))
        {
            _nextConnection = (index + 1) % _connections.size();
            return receiveFrom(_connections[index], session);
        }
    }

    return std::nullopt;
}

void NetworkManager::acceptSession()
{
    std::optional<TcpSocket> accepted = _server->accept();
    if (!accepted.has_value())
    {
        return;
    }

    const CommEndpoint remoteEndpoint = readRemoteEndpoint(*accepted);
    for (PeerConnection& connection : _connections)
    {
        if (connection.matches(remoteEndpoint))
        {
            connection.attach(std::move(*accepted));
            return;
        }
    }
    reportRejectedConnection(remoteEndpoint);
}

std::optional<ReceivedFrames> NetworkManager::receiveFrom(
    PeerConnection& connection,
    const std::shared_ptr<Session>& session
) {
    ReceivedFrames received{connection.getName(), {}};
    try
    {
        received.frames = session->receive();
    }
    //recv < 0 receive error 또는 framing error
    catch (const ReceiveError&)
    {
        if (!session->isOpen())
        {
            connection.detachIfCurrent(session);
        }
        throw;
    }

    //recv = 0 상대방 정상 연결 종료
    if (!session->isOpen())
    {
        connection.detachIfCurrent(session);
    }

    //recv = 0 또는 수신 byte < frame size 또는 would block
    if (received.frames.empty())
    {
        return std::nullopt;
    }

    return received;
}

namespace
{

bool contains(
    const std::vector<const TcpSocket*>& sockets,
    const TcpSocket& socket
) {
    return std::find(sockets.begin(), sockets.end(), &socket) != sockets.end();
}

CommEndpoint readRemoteEndpoint(const TcpSocket& socket)
{
    try
    {
        return socket.getRemoteEndpoint();
    }
    catch (const std::system_error& error)
    {
        throwSystemError<ConnectionError>(
            "Communication connection could not be established",
            "get-remote-endpoint",
            error.code().value());
    }
}

//올바르지 않은 주소에서 연결을 시도했을 때
void reportRejectedConnection(const CommEndpoint& remoteEndpoint)
{
    std::cerr
        << "[NetworkManager] Rejected unknown connection: remoteAddress="
        << remoteEndpoint.getAddress()
        << ", remotePort="
        << remoteEndpoint.getPort()
        << '\n';
}

} // namespace
} // namespace comm
