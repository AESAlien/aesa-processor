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
        CommEndpoint scenarioSimulator,
        CommEndpoint monitor
    );

    const CommEndpoint& server() const noexcept;
    const CommEndpoint& operatorConsole() const noexcept;
    const CommEndpoint& scenarioSimulator() const noexcept;
    const CommEndpoint& monitor() const noexcept;

private:
    CommEndpoint _server;
    CommEndpoint _operatorConsole;
    CommEndpoint _scenarioSimulator;
    CommEndpoint _monitor;
    void validate() const;
};

} // namespace comm
