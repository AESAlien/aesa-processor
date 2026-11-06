#pragma once

#include <comm/config/comm_config.hpp>
#include <network/connection_listener/connection_listener.hpp>
#include <network/network_manager/received_frames.hpp>
#include <network/peer_connection/peer_connection.hpp>

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace comm
{

// Owns configured connections and hides their lifetime and transport details.
class NetworkManager
{
public:
    explicit NetworkManager(CommConfig configuration);

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;
    NetworkManager(NetworkManager&&) = delete;
    NetworkManager& operator=(NetworkManager&&) = delete;

    // Throws ServerOpenError. Repeated calls are safe.
    void start();
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
    // The listener is destroyed before the connections it borrows.
    PeerConnection _operatorConsole;
    PeerConnection _scenarioSimulator;
    ConnectionListener _listener;
};

} // namespace comm
