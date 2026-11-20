#include <comm/controller/comm_controller.hpp>

#include <controller/comm_error_handler.hpp>
#include <message/message_decoder.hpp>
#include <message/message_encoder.hpp>
#include <network/network_manager.hpp>

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

CommController::CommController(
    CommConfig configuration,
    std::shared_ptr<spdlog::logger> messageLogger
) : _messageLogger(std::move(messageLogger)),
    _networkManager(std::make_unique<NetworkManager>(std::move(configuration))),
    _messageDecoder(std::make_unique<MessageDecoder>())
{
}

CommController::~CommController() = default;

void CommController::openServer()
{
    _networkManager->start();
}

std::optional<IncomingDto> CommController::receiveData()
{
    while (_pendingMessages.empty())
    {
        const std::optional<ReceivedFrames> received =
            _networkManager->poll(RECEIVE_POLL_TIMEOUT);

        if (received.has_value())
        {
            decodeFrames(*received);
        }
    }

    IncomingDto message = std::move(_pendingMessages.front());
    _pendingMessages.pop_front();
    return message;
}

void CommController::sendBeamCommand(
    const beam::TransmitBeamCommand& command
) {
    const EncodeResult result = encodeBeamCommand(command);
    if (!result.bytes)
    {
        controller_error::throwEncodingFailure("scenario-simulator", result.error);
    }
    _networkManager->sendToScenarioSimulator(*result.bytes);
}

void CommController::sendTrackEvents(
    const std::vector<target::TrackEvent>& trackEvents
) {
    const EncodeResult result = encodeTrackEvents(trackEvents);
    if (!result.bytes)
    {
        controller_error::throwEncodingFailure("operator-console", result.error);
    }
    _networkManager->sendToOperatorConsole(*result.bytes);
}

void CommController::sendRadarPose(
    const pose::SetRadarPoseCommand& command
) {
    const EncodeResult result = encodeRadarAttitude(command);
    if (!result.bytes)
    {
        controller_error::throwEncodingFailure("operator-console", result.error);
    }
    _networkManager->sendToOperatorConsole(*result.bytes);
}

// 오류가 발생한 프레임 이전에 디코딩한 메시지는 대기열에 유지한다.
void CommController::decodeFrames(const ReceivedFrames& received)
{
    for (const MessageFrame& frame : received.frames)
    {
        DecodeResult decodeResult = _messageDecoder->decode(frame);
        if (decodeResult.error != DecodingError::NONE ||
            !decodeResult.message.has_value())
        {
            controller_error::throwDecodingFailure(
                received.peerName,
                decodeResult.error
            );
        }

        _pendingMessages.push_back(std::move(*decodeResult.message));
    }
}

} // namespace comm
