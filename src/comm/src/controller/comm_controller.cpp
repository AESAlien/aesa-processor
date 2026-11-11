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

// 모니터는 보조 상대이므로 송신 대기를 짧게 제한한다.
constexpr std::chrono::milliseconds MONITOR_SEND_TIMEOUT{200};

const std::vector<uint8_t>& requireEncoded(
    const EncodeResult& result,
    const char* peerName
) {
    if (!result.bytes)
    {
        controller_error::throwEncodingFailure(peerName, result.error);
    }
    return *result.bytes;
}

} // namespace

class CommController::Impl
{
public:
    explicit Impl(CommConfig configuration)
      : _networkManager(std::move(configuration))
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
        const EncodeResult result = encodeBeamCommand(command);
        if (!result.bytes)
        {
            controller_error::throwEncodingFailure("scenario-simulator", result.error);
        }
        _networkManager.sendToScenarioSimulator(*result.bytes);
    }

    void sendTrackEvents(
        const std::vector<target::TrackEvent>& trackEvents
    ) {
        const EncodeResult result = encodeTrackEvents(trackEvents);
        if (!result.bytes)
        {
            controller_error::throwEncodingFailure("operator-console", result.error);
        }
        _networkManager.sendToOperatorConsole(*result.bytes);
    }

    void sendRadarAttitude(const beam::SetAttitudeCommand& command)
    {
        const EncodeResult result = encodeRadarAttitude(command);
        if (!result.bytes)
        {
            controller_error::throwEncodingFailure("operator-console", result.error);
        }
        _networkManager.sendToOperatorConsole(*result.bytes);
    }

    void sendToMonitor(const beam::TransmitBeamCommand& command)
    {
        _networkManager.sendToMonitor(
            requireEncoded(encodeBeamCommand(command), "monitor"),
            MONITOR_SEND_TIMEOUT
        );
    }

    void sendToMonitor(const std::vector<target::TrackEvent>& trackEvents)
    {
        _networkManager.sendToMonitor(
            requireEncoded(encodeTrackEvents(trackEvents), "monitor"),
            MONITOR_SEND_TIMEOUT
        );
    }

    void sendToMonitor(const beam::SetAttitudeCommand& command)
    {
        _networkManager.sendToMonitor(
            requireEncoded(encodeRadarAttitude(command), "monitor"),
            MONITOR_SEND_TIMEOUT
        );
    }

    void sendToMonitor(const MonitorStatusRequest& request)
    {
        MonitorStatus status;
        status.request = request;
        status.operatorConsoleConnected = _networkManager.isOperatorConsoleConnected();
        status.scenarioSimulatorConnected = _networkManager.isScenarioSimulatorConnected();

        _networkManager.sendToMonitor(
            requireEncoded(encodeMonitorStatus(status), "monitor"),
            MONITOR_SEND_TIMEOUT
        );
    }

private:
    // 오류가 발생한 프레임 이전에 디코딩한 메시지는 대기열에 유지한다.
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
                    decodeResult.error
                );
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
) {
    _impl->sendBeamCommand(command);
}

void CommController::sendTrackEvents(
    const std::vector<target::TrackEvent>& trackEvents
) {
    _impl->sendTrackEvents(trackEvents);
}

void CommController::sendRadarAttitude(
    const beam::SetAttitudeCommand& command
) {
    _impl->sendRadarAttitude(command);
}

void CommController::sendToMonitor(const beam::TransmitBeamCommand& command)
{
    _impl->sendToMonitor(command);
}

void CommController::sendToMonitor(const std::vector<target::TrackEvent>& trackEvents)
{
    _impl->sendToMonitor(trackEvents);
}

void CommController::sendToMonitor(const beam::SetAttitudeCommand& command)
{
    _impl->sendToMonitor(command);
}

void CommController::sendToMonitor(const MonitorStatusRequest& request)
{
    _impl->sendToMonitor(request);
}

} // namespace comm
