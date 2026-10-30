#pragma once

#include <network/native_socket/native_socket.hpp>
#include <network/network_manager/network_types.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

namespace comm
{

// IPv4 TCP server. Call every method except send() from the receive thread.
// send() may run concurrently with poll().
// An empty peer endpoint disables that peer. No message framing is performed.
class NetworkManager
{
public:
    NetworkManager(
        NetworkEndpoint serverEndpoint,
        NetworkEndpoint operatorConsoleEndpoint,
        NetworkEndpoint scenarioSimulatorEndpoint
    );

    ~NetworkManager();

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;
    NetworkManager(NetworkManager&&) = delete;
    NetworkManager& operator=(NetworkManager&&) = delete;

    OperationResult start();

    PollResult poll(std::chrono::milliseconds timeout);

    SendResult send(
        NetworkPeer peer,
        const std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{1000}
    );

    OperationResult stop();

private:
    using Clock = std::chrono::steady_clock;

    struct PeerSynchro
    {
        std::atomic<bool> closeRequested{false};
        std::mutex sendCloseMutex;
    };

    OperationResult validateConfiguration();
    bool acceptConnection(
        std::vector<NetworkEvent>& events,
        NetworkOperation& failedOperation,
        int& errorCode
    );
    void receiveFromSocket(
        SocketHandle socket,
        NetworkPeer peer,
        std::vector<NetworkEvent>& events
    );

    SocketHandle getSocket(NetworkPeer peer) const;
    void setSocket(NetworkPeer peer, SocketHandle socket);
    PeerSynchro& getPeerSynchro(NetworkPeer peer) const;
    int closeRequestedSockets();
    int closeSocket(NetworkPeer peer);
    int closeSocket(SocketHandle& socket);

    NetworkEndpoint _serverEndpoint;
    NetworkEndpoint _operatorConsoleEndpoint;
    NetworkEndpoint _scenarioSimulatorEndpoint;
    std::optional<uint32_t> _serverAddress;
    std::optional<uint32_t> _operatorConsoleAddress;
    std::optional<uint32_t> _scenarioSimulatorAddress;

    bool _socketApiStarted;
    SocketHandle _listenSocket;
    SocketHandle _operatorConsoleSocket;
    SocketHandle _scenarioSimulatorSocket;
    mutable PeerSynchro _operatorConsoleSynchro;
    mutable PeerSynchro _scenarioSimulatorSynchro;
};

} // namespace comm
