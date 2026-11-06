#pragma once

#include <comm/config/comm_endpoint.hpp>
#include <network/message_framer/message_framer.hpp>
#include <network/peer_connection/peer_connection.hpp>
#include <network/tcp_server/tcp_server.hpp>

#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <optional>
#include <vector>

namespace comm
{

struct ReceivedFrames
{
    std::string peerName;
    std::vector<MessageFrame> frames;
};

class NetworkManager
{
public:
    // Borrows connections, which must outlive this manager and its operations.
    NetworkManager(
        const CommEndpoint& serverEndpoint,
        std::vector<std::reference_wrapper<PeerConnection>> connections
    );

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;
    NetworkManager(NetworkManager&&) = delete;
    NetworkManager& operator=(NetworkManager&&) = delete;

    // ServerOpenError를 던짐
    void start();

    /*
        서버와 등록된 연결을 감시한다.
        새 연결을 수락하면 nullopt, 수신한 메시지가 완성되면 프레임을 반환한다.
    */
    std::optional<ReceivedFrames> poll(
        std::chrono::milliseconds timeout
    );

private:
    void acceptSession();
    std::optional<ReceivedFrames> receiveFrom(
        PeerConnection& connection,
        const std::shared_ptr<Session>& session
    );

    CommEndpoint _serverEndpoint;
    std::unique_ptr<TcpServer> _server;
    std::vector<std::reference_wrapper<PeerConnection>> _connections;
    std::size_t _nextConnection = 0;
};

} // namespace comm
