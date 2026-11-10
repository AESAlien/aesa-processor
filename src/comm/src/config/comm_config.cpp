#include <comm/config/comm_config.hpp>

#include <stdexcept>
#include <utility>

namespace comm
{
CommConfig::CommConfig(
    CommEndpoint server,
    CommEndpoint operatorConsole,
    CommEndpoint scenarioSimulator
)
  : _server(std::move(server)),
    _operatorConsole(std::move(operatorConsole)),
    _scenarioSimulator(std::move(scenarioSimulator))
{
    validate();
}

const CommEndpoint& CommConfig::server() const noexcept
{
    return _server;
}

const CommEndpoint& CommConfig::operatorConsole() const noexcept
{
    return _operatorConsole;
}

const CommEndpoint& CommConfig::scenarioSimulator() const noexcept
{
    return _scenarioSimulator;
}

void CommConfig::validate() const
{
    if (_operatorConsole == _scenarioSimulator)
    {
        throw std::invalid_argument(
            "Communication peer endpoints must be distinct"
        );
    }
}

} // namespace comm
