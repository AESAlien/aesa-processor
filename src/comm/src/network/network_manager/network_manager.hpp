#pragma once

#include <comm/config/comm_config.hpp>
#include <network/network_manager/received_frames.hpp>
#include <network/peer_connection/peer_connection.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace comm
{

class TcpServer;

// Owns configured connections and hides their lifetime and transport details.
// start() and poll() run on the receiving thread; sending may run concurrently.
class NetworkManager
{
public:
    explicit NetworkManager(CommConfig configuration);
    ~NetworkManager();

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;
    NetworkManager(NetworkManager&&) = delete;
    NetworkManager& operator=(NetworkManager&&) = delete;

    // Throws ServerOpenError. Repeated calls are safe.
    void start();

    // Watches the server and the current session of each peer.
    // Accepting a connection returns nullopt; completed frames are returned.
    // Throws ConnectionError or ReceiveError.
    std::optional<ReceivedFrames> poll(std::chrono::milliseconds timeout);

    // One sending thread per destination. Disconnected sends are ignored.
    // Throws SendError; rejects empty data and negative timeouts.
    void sendToOperatorConsole(
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );
    void sendToScenarioSimulator(
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );

private:
    // Hands a newly accepted session to the peer with the same endpoint.
    // Unknown connections are rejected and closed.
    void assignSession(const std::shared_ptr<Session>& session);

    std::optional<ReceivedFrames> receiveFrom(
        PeerConnection& connection,
        const std::shared_ptr<Session>& session
    );

    CommEndpoint _serverEndpoint;
    PeerConnection _operatorConsole;
    PeerConnection _scenarioSimulator;
    std::unique_ptr<TcpServer> _server;
};

} // namespace comm
