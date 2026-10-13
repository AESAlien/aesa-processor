#include <comm/controller/comm_controller.hpp>

#include <network/network_manager.hpp>

#include <iostream>
#include <mutex>
#include <utility>

namespace comm
{
namespace
{

NetworkEndpoint toNetworkEndpoint(CommEndpoint endpoint)
{
    return NetworkEndpoint{
        std::move(endpoint.address),
        endpoint.port
    };
}

const char* operationName(NetworkOperation operation)
{
    switch (operation)
    {
    case NetworkOperation::CONFIGURATION:
        return "configuration";

    case NetworkOperation::SOCKET_API_STARTUP:
        return "socket-api-startup";

    case NetworkOperation::CREATE_SOCKET:
        return "create-socket";

    case NetworkOperation::SET_SOCKET_OPTION:
        return "set-socket-option";

    case NetworkOperation::BIND:
        return "bind";

    case NetworkOperation::LISTEN:
        return "listen";

    default:
        return "unknown";
    }
}

void reportServerOpenFailure(
    const OperationResult& result
) noexcept
{
    try
    {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);

        std::cerr
            << "[CommController] Failed to open server: operation="
            << operationName(result.operation)
            << ", systemError="
            << result.errorCode
            << '\n';
    }
    catch (...)
    {
        // 오류 기록 실패가 원래 통신 오류를 대체하지 않도록 한다.
    }
}

} // namespace

class CommController::Impl
{
public:
    explicit Impl(CommConfiguration configuration)
        : _networkManager(
            toNetworkEndpoint(std::move(configuration.server)),
            toNetworkEndpoint(std::move(configuration.operatorConsole)),
            toNetworkEndpoint(std::move(configuration.scenarioSimulator)))
    {
    }

    void openServer()
    {
        const OperationResult result = _networkManager.start();
        if (result.status == OperationStatus::SUCCESS)
        {
            return;
        }

        reportServerOpenFailure(result);
        throw ServerOpenError(
            "Communication server could not be opened");
    }

private:
    NetworkManager _networkManager;
};

CommController::CommController(CommConfiguration configuration)
    : _impl(std::make_unique<Impl>(std::move(configuration)))
{
}

CommController::~CommController() = default;

void CommController::openServer()
{
    _impl->openServer();
}

} // namespace comm
