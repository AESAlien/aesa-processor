#pragma once

#include <comm/config/comm_config.hpp>
#include <network/network_manager/received_frames.hpp>
#include <network/peer_connection/peer_connection.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace comm
{

class TcpServer;

// 설정에 따라 만든 연결을 소유하고 수명 관리와 전송의 세부 구현을 감춘다.
// start()와 poll()은 수신 스레드에서 실행하며, 송신은 동시에 실행할 수 있다.
class NetworkManager
{
public:
    explicit NetworkManager(CommConfig configuration);
    ~NetworkManager();

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;
    NetworkManager(NetworkManager&&) = delete;
    NetworkManager& operator=(NetworkManager&&) = delete;

    // ServerOpenError를 던질 수 있다. 반복 호출해도 안전하다.
    void start();

    // 서버와 각 상대의 현재 세션을 감시한다.
    // 연결을 수락하면 nullopt를 반환하고, 프레임이 완성되면 해당 프레임들을 반환한다.
    // ConnectionError 또는 ReceiveError를 던질 수 있다.
    std::optional<ReceivedFrames> poll(std::chrono::milliseconds timeout);

    // 목적지마다 송신 스레드는 하나만 사용한다. 연결이 끊긴 상태의 송신은 무시한다.
    // SendError를 던질 수 있다. 빈 데이터와 음수 타임아웃은 거부한다.
    void sendToOperatorConsole(
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );
    void sendToScenarioSimulator(
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );

private:
    static constexpr std::size_t PEER_COUNT = 2;

    std::array<PeerConnection*, PEER_COUNT> peers() noexcept;

    // 새로 수락한 세션을 엔드포인트가 같은 상대에게 전달한다.
    // 알 수 없는 상대의 연결은 거부하고 닫는다.
    void assignToPeer(const std::shared_ptr<Session>& session);

    std::optional<ReceivedFrames> receiveFrom(PeerConnection& peer);

    CommEndpoint _serverEndpoint;
    PeerConnection _operatorConsole;
    PeerConnection _scenarioSimulator;
    std::unique_ptr<TcpServer> _server;
};

} // namespace comm
