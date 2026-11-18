#include <network/network_manager.hpp>

#include <comm/error/comm_error.hpp>
#include <network/tcp_server.hpp>

#include <iostream>
#include <string>
#include <utility>

namespace comm
{
namespace
{

std::string describe(const CommEndpoint& endpoint)
{
    return endpoint.getAddress() + ":" + std::to_string(endpoint.getPort());
}

} // namespace

NetworkManager::NetworkManager(
    CommConfig configuration,
    std::shared_ptr<spdlog::logger> logger
) : _logger(std::move(logger)),
    _serverEndpoint(configuration.server()),
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

    // 현재 세션만 감시한다. 송신 측이 교체된 이전 세션을 계속 보유하더라도
    // 다시 감시하지 않으므로 새 세션으로 잘못 처리하지 않는다.
    // currentSessions[index]는 peers()[index]에 속한다.
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
            // 같은 상대가 이전 연결을 닫지 않은 채 다시 접속하면 이전 세션을 새 세션으로 바꾼다.
            const bool replaced = static_cast<bool>(peer->getSession());
            peer->setSession(session);
            if (_logger)
            {
                _logger->info(
                    "Peer connected: peer={}, remote={}{}",
                    peer->getName(),
                    describe(session->getRemoteEndpoint()),
                    replaced ? ", replacedPreviousSession=true" : ""
                );
            }
            return;
        }
    }

    // 반환 시 거부한 세션을 해제하고 해당 연결을 닫는다.
    // 주소는 같고 포트만 다른 재접속(예: 운용콘솔이 3100이 아닌 포트로 접속)도 여기서 거부된다.
    const CommEndpoint& remoteEndpoint = session->getRemoteEndpoint();
    if (_logger)
    {
        _logger->warn(
            "Rejected unknown connection: remoteAddress={}, remotePort={}",
            remoteEndpoint.getAddress(),
            remoteEndpoint.getPort()
        );
        return;
    }
    std::cerr << "[NetworkManager] Rejected unknown connection: remoteAddress="
              << remoteEndpoint.getAddress()
              << ", remotePort=" << remoteEndpoint.getPort() << '\n';
}

std::optional<ReceivedFrames> NetworkManager::receiveFrom(PeerConnection& peer)
{
    const std::shared_ptr<Session> session = peer.getSession();
    std::vector<MessageFrame> frames;
    try
    {
        frames = peer.receive();
    }
    catch (const ReceiveError&)
    {
        logIfClosed(peer, session, "receive-error");
        throw;
    }
    logIfClosed(peer, session, "closed-by-peer");

    if (frames.empty())
    {
        return std::nullopt;
    }
    return ReceivedFrames{peer.getName(), std::move(frames)};
}

void NetworkManager::logIfClosed(
    const PeerConnection& peer,
    const std::shared_ptr<Session>& session,
    const char* reason
) const {
    if (!_logger || !session || session->isOpen())
    {
        return;
    }
    // 송신 실패로 이쪽에서 닫은 연결은 수신 경로가 EOF나 오류로 확인하더라도 상대가 끊은 것이 아니다.
    _logger->info(
        "Peer disconnected: peer={}, remote={}, reason={}",
        peer.getName(),
        describe(session->getRemoteEndpoint()),
        session->wasShutDownAfterSendFailure() ? "send-error" : reason
    );
}

} // namespace comm
