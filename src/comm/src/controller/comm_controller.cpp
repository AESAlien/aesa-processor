#include <comm/controller/comm_controller.hpp>

#include <controller/comm_error_handler.hpp>
#include <network/message_decoder/message_decoder.hpp>
#include <network/message_encoder/message_encoder.hpp>
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
        endpoint.address(),
        endpoint.port()
    };
}

} // namespace

class CommController::Impl
{
public:
    explicit Impl(CommConfig configuration)
        : _networkManager(
            toNetworkEndpoint(configuration.server()),
            toNetworkEndpoint(configuration.operatorConsole()),
            toNetworkEndpoint(configuration.scenarioSimulator()))
    {
    }

    void openServer()
    {
        _networkManager.start();
    }

    std::optional<IncomingDto> receiveData()
    {
        while (_pendingMessages.empty())
        {
            const std::optional<ReceivedFrames> received =
                _networkManager.receiveFrames(RECEIVE_POLL_TIMEOUT);

            if (received.has_value())
            {
                decodeFrames(*received);
            }
        }

        IncomingDto message = std::move(_pendingMessages.front());
        _pendingMessages.pop_front();
        return message;
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

        _networkManager.send(peer, *encodeResult.bytes);
    }

    // Messages decoded before a failing frame stay queued.
    void decodeFrames(const ReceivedFrames& received)
    {
        for (const MessageFrame& frame : received.frames)
        {
            DecodeResult decodeResult = _messageDecoder.decode(frame);
            if (decodeResult.error != DecodingError::NONE ||
                !decodeResult.message.has_value())
            {
                controller_error::throwDecodingFailure(
                    received.peer,
                    decodeResult.error);
            }

            _pendingMessages.push_back(std::move(*decodeResult.message));
        }
    }

    NetworkManager _networkManager;
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
