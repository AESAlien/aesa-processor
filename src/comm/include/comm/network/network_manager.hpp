#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace comm
{

enum class NetworkPeer
{
    OPERATOR_CONSOLE,
    SCENARIO_SIMULATOR
};

struct ReceivedData
{
    NetworkPeer peer;
    std::vector<std::uint8_t> bytes;
};

class NetworkManager
{
public:
    NetworkManager(
        std::uint16_t port,
        const std::string& operatorConsoleIp,
        const std::string& scenarioSimulatorIp);

    ~NetworkManager();

    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;

    bool start();
    bool receive(ReceivedData& receivedData, int timeoutMilliseconds);
    bool send(NetworkPeer peer, const std::vector<std::uint8_t>& data);

    bool isConnected(NetworkPeer peer) const;

    void disconnect(NetworkPeer peer);
    void stop();

private:
    bool acceptConnection();
    bool receiveFromSocket(
        int socketFd,
        NetworkPeer peer,
        ReceivedData& receivedData);

    bool sendAll(
        int socketFd,
        const std::vector<std::uint8_t>& data);

    int getSocket(NetworkPeer peer) const;
    void setSocket(NetworkPeer peer, int socketFd);
    void closeSocket(int& socketFd);

    std::uint16_t _port;
    std::string _operatorConsoleIp;
    std::string _scenarioSimulatorIp;

    int _listenSocket;
    int _operatorConsoleSocket;
    int _scenarioSimulatorSocket;
};

} // namespace comm
