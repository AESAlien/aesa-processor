#pragma once

#include <network/message_framer/message_framer.hpp>
#include <network/network_manager/network_types.hpp>
#include <network/session/session.hpp>
#include <network/tcp_server/tcp_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace comm
{

struct ReceivedFrames
{
    NetworkPeer peer;
    std::vector<MessageFrame> frames;
};



class NetworkManager
{
public:
    // Throws std::invalid_argument for an invalid peer configuration.
    NetworkManager(
        NetworkEndpoint serverEndpoint,
        NetworkEndpoint operatorConsoleEndpoint,
        NetworkEndpoint scenarioSimulatorEndpoint
    );

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;
    NetworkManager(NetworkManager&&) = delete;
    NetworkManager& operator=(NetworkManager&&) = delete;

    // ServerOpenError를 던짐
    void start();

    /*
        timeout동안 등록된 소켓들을 감시하고, 반환된 소켓을 처리한다.
        listen 소켓을 반환했을 때 주소 일치 시, 신규 연결 시 accept, 
        재연결 시 reconnect, 주소 불일치 시 reject
        operator, simulator 소켓을 반환했을 때는 framing 하여 반환한다.
    */

    // Throws ConnectionError or ReceiveError.
    std::optional<ReceivedFrames> receiveFrames(
        std::chrono::milliseconds timeout
    );

    // Does nothing while the peer is not connected. Throws SendError.
    void send(
        NetworkPeer peer,
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );

private:
    //CommConfiguration이 올바른지 검사
    void validatePeers() const;
    //accpet 하기 전 올바른 주소인지 검사
    std::optional<NetworkPeer> identifyPeer(
        const AcceptedSocket& accepted
    ) const;

    void acceptSession();
    std::optional<ReceivedFrames> receiveFrom(
        NetworkPeer peer,
        const std::shared_ptr<Session>& session
    );

    std::shared_ptr<Session> getSession(NetworkPeer peer) const;
    void setSession(NetworkPeer peer, std::shared_ptr<Session> session);

    Ipv4Endpoint _serverEndpoint;
    std::optional<Ipv4Endpoint> _operatorConsoleEndpoint;     // nullopt = 비활성
    std::optional<Ipv4Endpoint> _scenarioSimulatorEndpoint;   // nullopt = 비활성

    
    // Declared before the sessions so that it is destroyed after them.
    std::unique_ptr<TcpServer> _server;

    // Guards replacing the session pointers against send() copying them.
    mutable std::mutex _sessionMutex;
    std::shared_ptr<Session> _operatorConsoleSession;
    std::shared_ptr<Session> _scenarioSimulatorSession;

    NetworkPeer _nextPeer = NetworkPeer::OPERATOR_CONSOLE;
};

} // namespace comm
