#include <network/network_manager/network_manager.hpp>

#include <comm/error/comm_error.hpp>
#include <network/tcp_server/tcp_server.hpp>

#include <iostream>
#include <utility>

namespace comm
{

NetworkManager::NetworkManager(CommConfig configuration)
  : _serverEndpoint(configuration.server()),
    _operatorConsole("operator-console", configuration.operatorConsole()),
    _scenarioSimulator("scenario-simulator", configuration.scenarioSimulator())
{
}

NetworkManager::~NetworkManager() = default;

void NetworkManager::start()
{
    if (!_server)
    {
        _server = std::make_unique<TcpServer>(_serverEndpoint);
    }
}

std::optional<ReceivedFrames> NetworkManager::poll(std::chrono::milliseconds timeout)
{
    if (!_server)
    {
        throw ReceiveError("Communication data could not be received: server is not started");
    }

    // Only the current sessions are watched. A replaced session that a sender
    // still holds is never watched again, so it cannot be taken for a new one.
    // currentSessions[index] belongs to peers()[index].
    const std::array<PeerConnection*, PEER_COUNT> connections = peers();
    std::vector<std::shared_ptr<Session>> currentSessions;
    for (PeerConnection* peer : connections)
    {
        currentSessions.push_back(peer->getSession());
    }

    const ServerEvent event = _server->poll(currentSessions, timeout);
    switch (event.kind)
    {
    case ServerEventKind::NEW_SESSION:
        assignToPeer(event.session);
        return std::nullopt;

    case ServerEventKind::READABLE:
        for (std::size_t index = 0; index < PEER_COUNT; ++index)
        {
            if (currentSessions[index] == event.session)
            {
                return receiveFrom(*connections[index]);
            }
        }
        return std::nullopt;

    case ServerEventKind::NONE:
        return std::nullopt; 
    }

    return std::nullopt;
}

void NetworkManager::sendToOperatorConsole(
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
) {
    _operatorConsole.send(data, timeout);
}

void NetworkManager::sendToScenarioSimulator(
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
) {
    _scenarioSimulator.send(data, timeout);
}

std::array<PeerConnection*, NetworkManager::PEER_COUNT> NetworkManager::peers() noexcept
{
    return {&_operatorConsole, &_scenarioSimulator};
}

void NetworkManager::assignToPeer(const std::shared_ptr<Session>& session)
{
    for (PeerConnection* peer : peers())
    {
        if (peer->matches(*session))
        {
            peer->setSession(session);
            return;
        }
    }

    // The rejected session is released, and its connection closed, on return.
    const CommEndpoint& remoteEndpoint = session->getRemoteEndpoint();
    std::cerr << "[NetworkManager] Rejected unknown connection: remoteAddress="
              << remoteEndpoint.getAddress()
              << ", remotePort=" << remoteEndpoint.getPort() << '\n';
}

std::optional<ReceivedFrames> NetworkManager::receiveFrom(PeerConnection& peer)
{
    std::vector<MessageFrame> frames = peer.receive();
    if (frames.empty())
    {
        return std::nullopt;
    }
    return ReceivedFrames{peer.getName(), std::move(frames)};
}

} // namespace comm
