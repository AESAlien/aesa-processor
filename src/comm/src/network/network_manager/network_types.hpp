#pragma once

#include <cstdint>
#include <string>

namespace comm
{

//CommConfiguration에서 받는 문자열 값
struct NetworkEndpoint
{
    std::string address;
    uint16_t port = 0;
};

// CommConfiguration의 문자열을 정수형으로 변환
struct Ipv4Endpoint
{
    uint32_t address;   // network byte order
    uint16_t port;
};

enum class NetworkPeer
{
    OPERATOR_CONSOLE,
    SCENARIO_SIMULATOR
};

inline const char* peerName(NetworkPeer peer)
{
    switch (peer)
    {
    case NetworkPeer::OPERATOR_CONSOLE:
        return "operator-console";

    case NetworkPeer::SCENARIO_SIMULATOR:
        return "scenario-simulator";
    }

    return "unknown";
}

} // namespace comm
