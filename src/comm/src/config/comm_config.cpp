#include <comm/config/comm_config.hpp>

#include <stdexcept>
#include <utility>

namespace comm
{
CommConfig::CommConfig(
    CommEndpoint server,
    CommEndpoint operatorConsole,
    CommEndpoint scenarioSimulator,
    CommEndpoint monitor
)
  : _server(std::move(server)),
    _operatorConsole(std::move(operatorConsole)),
    _scenarioSimulator(std::move(scenarioSimulator)),
    _monitor(std::move(monitor))
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

const CommEndpoint& CommConfig::monitor() const noexcept
{
    return _monitor;
}

void CommConfig::validate() const
{
    // 수락한 연결을 엔드포인트로 상대에게 배정하므로 상대마다 엔드포인트가 달라야 한다.
    if (_operatorConsole == _scenarioSimulator ||
        _operatorConsole == _monitor ||
        _scenarioSimulator == _monitor)
    {
        throw std::invalid_argument(
            "Communication peer endpoints must be distinct"
        );
    }
}

} // namespace comm
