#pragma once

#include <comm/config/comm_endpoint.hpp>

namespace comm
{

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
