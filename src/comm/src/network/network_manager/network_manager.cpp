#include <network/network_manager/network_manager.hpp>

namespace comm
{

NetworkManager::NetworkManager(CommConfig configuration)
  : _operatorConsole("operator-console", configuration.operatorConsole()),
    _scenarioSimulator("scenario-simulator", configuration.scenarioSimulator()),
    _listener(configuration.server(), {_operatorConsole, _scenarioSimulator})
{
}

void NetworkManager::start()
{
    _listener.start();
}

std::optional<ReceivedFrames> NetworkManager::poll(std::chrono::milliseconds timeout)
{
    return _listener.poll(timeout);
}

void NetworkManager::sendToOperatorConsole(
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
) {
    _operatorConsole.send(data, timeout);
}

void NetworkManager::sendToScenarioSimulator(
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
) {
    _scenarioSimulator.send(data, timeout);
}

} // namespace comm
