#pragma once

#include <cstdint>
#include <string>

namespace comm
{

class CommEndpoint
{
public:
    CommEndpoint(std::string address, std::uint16_t port);

    const std::string& address() const noexcept;
    std::uint16_t port() const noexcept;

private:
    std::string _address;
    std::uint16_t _port;
    void validate() const;
};

class CommConfig
{
public:
    CommConfig(
        CommEndpoint server,
        CommEndpoint operatorConsole,
        CommEndpoint scenarioSimulator);

    const CommEndpoint& server() const noexcept;
    const CommEndpoint& operatorConsole() const noexcept;
    const CommEndpoint& scenarioSimulator() const noexcept;

private:
    CommEndpoint _server;
    CommEndpoint _operatorConsole;
    CommEndpoint _scenarioSimulator;
    void validate() const;
};

} // namespace comm
