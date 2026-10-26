#pragma once

#include <cstdint>
#include <string>

namespace comm
{

struct CommEndpoint
{
    CommEndpoint(std::string address, std::uint16_t port);

    bool isDisabled() const noexcept;

    const std::string address;
    const std::uint16_t port;
};

struct CommConfiguration
{
    CommConfiguration(
        CommEndpoint server,
        CommEndpoint operatorConsole,
        CommEndpoint scenarioSimulator);

    CommEndpoint server;
    CommEndpoint operatorConsole;
    CommEndpoint scenarioSimulator;
};

} // namespace comm
