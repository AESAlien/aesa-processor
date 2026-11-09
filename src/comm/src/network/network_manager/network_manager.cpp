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
    std::vector<std::shared_ptr<Session>> currentSessions;
    for (PeerConnection* peer : peers())
    {
        currentSessions.push_back(peer->snapshotSession());
    }

    const ServerEvent event = _server->poll(currentSessions, timeout);
    switch (event.kind)
    {
    case ServerEventKind::NEW_SESSION:
        assignSession(event.session);
        return std::nullopt;

    case ServerEventKind::READABLE:
        return receiveFrom(event.session);

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

void NetworkManager::assignSession(const std::shared_ptr<Session>& session)
{
    const CommEndpoint& remoteEndpoint = session->getRemoteEndpoint();
    for (PeerConnection* peer : peers())
    {
        if (peer->matches(remoteEndpoint))
        {
            peer->replaceSession(session);
            return;
        }
    }

    // The rejected session is released, and its connection closed, on return.
    std::cerr << "[NetworkManager] Rejected unknown connection: remoteAddress="
              << remoteEndpoint.getAddress()
              << ", remotePort=" << remoteEndpoint.getPort() << '\n';
}

std::optional<ReceivedFrames> NetworkManager::receiveFrom(const std::shared_ptr<Session>& session)
{
    for (PeerConnection* peer : peers())
    {
        // Only the peer whose current session it is receives; others return nullopt.
        std::optional<std::vector<MessageFrame>> frames = peer->receiveIfCurrent(session);
        if (frames)
        {
            if (frames->empty())
            {
                return std::nullopt;
            }
            return ReceivedFrames{peer->getName(), std::move(*frames)};
        }
    }
    return std::nullopt;
}

} // namespace comm
