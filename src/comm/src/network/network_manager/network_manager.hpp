#pragma once

#include <comm/config/comm_config.hpp>
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
        const CommEndpoint& serverEndpoint,
        const CommEndpoint& operatorConsoleEndpoint,
        const CommEndpoint& scenarioSimulatorEndpoint
    );

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;
    NetworkManager(NetworkManager&&) = delete;
    NetworkManager& operator=(NetworkManager&&) = delete;

    // ServerOpenError를 던짐
    void start();

    
    /*
        listen socket과 존재하는 console, simulator socket을 감시
        new socket accept -> nullopt
        console or simulator byte recv -> framing 후 frame 반환
    */
    std::optional<ReceivedFrames> poll(
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
    Ipv4Endpoint _operatorConsoleEndpoint;
    Ipv4Endpoint _scenarioSimulatorEndpoint;

    
    // Declared before the sessions so that it is destroyed after them.
    std::unique_ptr<TcpServer> _server;

    // Guards replacing the session pointers against send() copying them.
    mutable std::mutex _sessionMutex;
    std::shared_ptr<Session> _operatorConsoleSession;
    std::shared_ptr<Session> _scenarioSimulatorSession;

    NetworkPeer _nextPeer = NetworkPeer::OPERATOR_CONSOLE;
};

} // namespace comm
