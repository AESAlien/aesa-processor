#pragma once

#include <cstdint>
#include <string>

namespace comm
{

struct CommEndpoint
{
    CommEndpoint(std::string address, std::uint16_t port);

    const std::string address;
    const std::uint16_t port;
};

struct CommConfig
{
    CommConfig(
        CommEndpoint server,
        CommEndpoint operatorConsole,
        CommEndpoint scenarioSimulator);

    const CommEndpoint server;
    const CommEndpoint operatorConsole;
    const CommEndpoint scenarioSimulator;
};

} // namespace comm
