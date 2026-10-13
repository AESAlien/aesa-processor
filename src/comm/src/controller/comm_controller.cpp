#include <comm/controller/comm_controller.hpp>

#include <network/message_decoder.hpp>
#include <network/message_framer.hpp>
#include <network/network_manager.hpp>

#include <chrono>
#include <deque>
#include <iostream>
#include <mutex>
#include <utility>
#include <vector>

namespace comm
{
namespace
{

constexpr std::chrono::milliseconds RECEIVE_POLL_TIMEOUT{100};

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

    case NetworkOperation::SELECT:
        return "select";

    case NetworkOperation::ACCEPT:
        return "accept";

    case NetworkOperation::ADDRESS_CONVERSION:
        return "address-conversion";

    case NetworkOperation::RECEIVE:
        return "receive";

    case NetworkOperation::SEND:
        return "send";

    case NetworkOperation::CLOSE:
        return "close";

    case NetworkOperation::SOCKET_API_CLEANUP:
        return "socket-api-cleanup";

    case NetworkOperation::NONE:
        return "none";

    default:
        return "unknown";
    }
}

const char* peerName(NetworkPeer peer)
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

const char* framingErrorName(FramingError error)
{
    switch (error)
    {
    case FramingError::NONE:
        return "none";

    case FramingError::UNKNOWN_MESSAGE_ID:
        return "unknown-message-id";

    case FramingError::INVALID_FRAME_SIZE:
        return "invalid-frame-size";
    }

    return "unknown";
}

const char* decodingErrorName(DecodingError error)
{
    switch (error)
    {
    case DecodingError::NONE:
        return "none";

    case DecodingError::UNSUPPORTED_MESSAGE_ID:
        return "unsupported-message-id";

    case DecodingError::UNSUPPORTED_VERSION:
        return "unsupported-version";

    case DecodingError::INVALID_MESSAGE_SIZE:
        return "invalid-message-size";

    case DecodingError::INVALID_ITEM_COUNT:
        return "invalid-item-count";

    case DecodingError::INVALID_FIELD_VALUE:
        return "invalid-field-value";
    }

    return "unknown";
}

std::mutex& logMutex()
{
    static std::mutex mutex;
    return mutex;
}

void reportNetworkFailure(
    const char* action,
    NetworkOperation operation,
    int errorCode
) noexcept
{
    try
    {
        std::lock_guard<std::mutex> lock(logMutex());

        std::cerr
            << "[CommController] Failed to "
            << action
            << ": operation="
            << operationName(operation)
            << ", systemError="
            << errorCode
            << '\n';
    }
    catch (...)
    {
        // 오류 기록 실패가 원래 통신 오류를 대체하지 않도록 한다.
    }
}

void reportProtocolFailure(
    const char* stage,
    NetworkPeer peer,
    const char* error
) noexcept
{
    try
    {
        std::lock_guard<std::mutex> lock(logMutex());

        std::cerr
            << "[CommController] Failed to process received data: stage="
            << stage
            << ", peer="
            << peerName(peer)
            << ", error="
            << error
            << '\n';
    }
    catch (...)
    {
        // 오류 기록 실패가 원래 통신 오류를 대체하지 않도록 한다.
    }
}

void reportRejectedConnection(
    const ConnectionRejectedEvent& event
) noexcept
{
    try
    {
        std::lock_guard<std::mutex> lock(logMutex());

        std::cerr
            << "[CommController] Rejected unknown connection: remoteAddress="
            << event.remoteAddress
            << ", remotePort="
            << event.remotePort
            << '\n';
    }
    catch (...)
    {
        // 오류 기록 실패가 원래 통신 흐름을 방해하지 않도록 한다.
    }
}

bool isConnectionOperation(NetworkOperation operation)
{
    return operation == NetworkOperation::ACCEPT ||
        operation == NetworkOperation::SET_SOCKET_OPTION ||
        operation == NetworkOperation::ADDRESS_CONVERSION;
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

        reportNetworkFailure(
            "open server",
            result.operation,
            result.errorCode);
        throw ServerOpenError(
            "Communication server could not be opened");
    }

    std::optional<IncomingDto> receiveData()
    {
        if (!_pendingMessages.empty())
        {
            return takePendingMessage();
        }

        while (true)
        {
            const PollResult result =
                _networkManager.poll(RECEIVE_POLL_TIMEOUT);

            switch (result.status)
            {
            case PollStatus::EVENTS:
                processEvents(result.events);
                if (!_pendingMessages.empty())
                {
                    return takePendingMessage();
                }
                break;

            case PollStatus::TIMEOUT:
                break;

            case PollStatus::ERROR:
                handlePollFailure(result);
                break;

            case PollStatus::NOT_STARTED:
            case PollStatus::INVALID_ARGUMENT:
                reportNetworkFailure(
                    "receive data",
                    result.failedOperation,
                    result.errorCode);
                throw ReceiveError(
                    "Communication data could not be received");
            }
        }
    }

private:
    std::optional<IncomingDto> takePendingMessage()
    {
        IncomingDto message = std::move(_pendingMessages.front());
        _pendingMessages.pop_front();
        return message;
    }

    MessageFramer& getFramer(NetworkPeer peer)
    {
        switch (peer)
        {
        case NetworkPeer::OPERATOR_CONSOLE:
            return _operatorConsoleFramer;

        case NetworkPeer::SCENARIO_SIMULATOR:
            return _scenarioSimulatorFramer;
        }

        return _operatorConsoleFramer;
    }

    void handlePollFailure(const PollResult& result)
    {
        if (isConnectionOperation(result.failedOperation))
        {
            reportNetworkFailure(
                "accept connection",
                result.failedOperation,
                result.errorCode);
            throw ConnectionError(
                "Communication connection could not be established");
        }

        reportNetworkFailure(
            "receive data",
            result.failedOperation,
            result.errorCode);
        throw ReceiveError(
            "Communication data could not be received");
    }

    void processEvents(const std::vector<NetworkEvent>& events)
    {
        std::vector<IncomingDto> decodedMessages;

        for (const NetworkEvent& event : events)
        {
            if (const auto* connection =
                    std::get_if<ConnectionEvent>(&event))
            {
                processConnectionEvent(*connection);
                continue;
            }

            if (const auto* data =
                    std::get_if<DataReceivedEvent>(&event))
            {
                decodeData(*data, decodedMessages);
                continue;
            }

            reportRejectedConnection(
                std::get<ConnectionRejectedEvent>(event));
        }

        for (IncomingDto& message : decodedMessages)
        {
            _pendingMessages.push_back(std::move(message));
        }
    }

    void processConnectionEvent(const ConnectionEvent& event)
    {
        MessageFramer& framer = getFramer(event.peer);
        framer.reset();

        if (event.state != ConnectionState::DISCONNECTED ||
            event.disconnectReason != DisconnectReason::RECEIVE_ERROR)
        {
            return;
        }

        reportNetworkFailure(
            "receive data",
            NetworkOperation::RECEIVE,
            event.errorCode);
        throw ReceiveError(
            "Communication data could not be received");
    }

    void decodeData(
        const DataReceivedEvent& event,
        std::vector<IncomingDto>& decodedMessages
    )
    {
        MessageFramer& framer = getFramer(event.peer);
        FrameResult frameResult = framer.append(event.bytes);

        if (frameResult.error != FramingError::NONE)
        {
            reportProtocolFailure(
                "framing",
                event.peer,
                framingErrorName(frameResult.error));
            framer.reset();
            throw ReceiveError(
                "Communication data could not be received");
        }

        for (const MessageFrame& frame : frameResult.frames)
        {
            DecodeResult decodeResult = _messageDecoder.decode(frame);
            if (decodeResult.error != DecodingError::NONE ||
                !decodeResult.message.has_value())
            {
                reportProtocolFailure(
                    "decoding",
                    event.peer,
                    decodingErrorName(decodeResult.error));
                framer.reset();
                throw ReceiveError(
                    "Communication data could not be received");
            }

            decodedMessages.push_back(
                std::move(*decodeResult.message));
        }
    }

    NetworkManager _networkManager;
    MessageFramer _operatorConsoleFramer;
    MessageFramer _scenarioSimulatorFramer;
    MessageDecoder _messageDecoder;
    std::deque<IncomingDto> _pendingMessages;
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

std::optional<IncomingDto> CommController::receiveData()
{
    return _impl->receiveData();
}

} // namespace comm
