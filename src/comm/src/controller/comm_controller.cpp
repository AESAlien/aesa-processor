#include <comm/controller/comm_controller.hpp>

#include <controller/comm_error_handler.hpp>
#include <network/message_decoder/message_decoder.hpp>
#include <network/message_encoder/message_encoder.hpp>
#include <network/message_framer/message_framer.hpp>
#include <network/network_manager/network_manager.hpp>

#include <chrono>
#include <deque>
#include <utility>
#include <vector>

namespace comm
{
namespace
{

constexpr std::chrono::milliseconds RECEIVE_POLL_TIMEOUT{100};

NetworkEndpoint toNetworkEndpoint(const CommEndpoint& endpoint)
{
    return NetworkEndpoint{
        endpoint.address,
        endpoint.port
    };
}

} // namespace

class CommController::Impl
{
public:
    explicit Impl(CommConfig configuration)
        : _networkManager(
            toNetworkEndpoint(configuration.server),
            toNetworkEndpoint(configuration.operatorConsole),
            toNetworkEndpoint(configuration.scenarioSimulator))
    {
    }

    void openServer()
    {
        const OperationResult result = _networkManager.start();
        if (result.status == OperationStatus::SUCCESS)
        {
            return;
        }

        controller_error::throwServerOpenFailure(result);
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
                controller_error::throwPollFailure(result);
                break;

            case PollStatus::NOT_STARTED:
            case PollStatus::INVALID_ARGUMENT:
                controller_error::throwReceiveFailure(
                    result.failedOperation,
                    result.errorCode);
            }
        }
    }

    void sendBeamCommand(const beam::TransmitBeamCommand& command)
    {
        sendEncodedMessage(
            NetworkPeer::SCENARIO_SIMULATOR,
            encodeBeamCommand(command));
    }

    void sendTrackEvents(
        const std::vector<target::TrackEvent>& trackEvents
    )
    {
        sendEncodedMessage(
            NetworkPeer::OPERATOR_CONSOLE,
            encodeTrackEvents(trackEvents));
    }

private:
    void sendEncodedMessage(
        NetworkPeer peer,
        const EncodeResult& encodeResult
    )
    {
        if (!encodeResult.bytes.has_value())
        {
            controller_error::throwEncodingFailure(
                peer,
                encodeResult.error);
        }

        const SendResult sendResult = _networkManager.send(
            peer,
            *encodeResult.bytes);

        if (sendResult.status == SendStatus::SUCCESS ||
            sendResult.status == SendStatus::NOT_CONNECTED)
        {
            return;
        }

        controller_error::throwSendFailure(peer, sendResult);
    }

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

            controller_error::reportRejectedConnection(
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

        controller_error::throwReceiveFailure(
            NetworkOperation::RECEIVE,
            event.errorCode);
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
            framer.reset();
            controller_error::throwFramingFailure(
                event.peer,
                frameResult.error);
        }

        for (const MessageFrame& frame : frameResult.frames)
        {
            DecodeResult decodeResult = _messageDecoder.decode(frame);
            if (decodeResult.error != DecodingError::NONE ||
                !decodeResult.message.has_value())
            {
                framer.reset();
                controller_error::throwDecodingFailure(
                    event.peer,
                    decodeResult.error);
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

CommController::CommController(CommConfig configuration)
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

void CommController::sendBeamCommand(
    const beam::TransmitBeamCommand& command
)
{
    _impl->sendBeamCommand(command);
}

void CommController::sendTrackEvents(
    const std::vector<target::TrackEvent>& trackEvents
)
{
    _impl->sendTrackEvents(trackEvents);
}

} // namespace comm
