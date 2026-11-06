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

} // namespace

class CommController::Impl
{
public:
    explicit Impl(CommConfig configuration)
        : _operatorConsole("operator-console", configuration.operatorConsole()),
          _scenarioSimulator("scenario-simulator", configuration.scenarioSimulator()),
          _networkManager(
              configuration.server(),
              {_operatorConsole, _scenarioSimulator})
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
                _networkManager.poll(RECEIVE_POLL_TIMEOUT);

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
            _scenarioSimulator,
            encodeBeamCommand(command)
        );
    }

    void sendTrackEvents(
        const std::vector<target::TrackEvent>& trackEvents
    ) {
        sendEncodedMessage(
            _operatorConsole,
            encodeTrackEvents(trackEvents)
        );
    }

private:
    void sendEncodedMessage(
        PeerConnection& connection,
        const EncodeResult& encodeResult
    ) {
        if (!encodeResult.bytes.has_value())
        {
            controller_error::throwEncodingFailure(
                connection.getName(),
                encodeResult.error);
        }

        connection.send(*encodeResult.bytes);
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
                    received.peerName,
                    decodeResult.error);
            }

            _pendingMessages.push_back(std::move(*decodeResult.message));
        }
    }

    // Connections outlive the manager that borrows them (reverse destruction order).
    PeerConnection _operatorConsole;
    PeerConnection _scenarioSimulator;
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
) {
    _impl->sendTrackEvents(trackEvents);
}

} // namespace comm
