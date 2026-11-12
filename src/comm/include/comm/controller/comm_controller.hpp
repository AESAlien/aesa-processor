#pragma once

#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/transmit_beam_command.hpp>
#include <comm/config/comm_config.hpp>
#include <comm/dto/incoming_dto.hpp>
#include <comm/dto/monitor_status_request.hpp>
#include <comm/error/comm_error.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace comm
{

class CommController
{
public:
    explicit CommController(CommConfig configuration);
    ~CommController();

    CommController(const CommController&) = delete;
    CommController& operator=(const CommController&) = delete;
    CommController(CommController&&) = delete;
    CommController& operator=(CommController&&) = delete;

    // 실패하면 상세 시스템 오류를 메시지에 포함한 ServerOpenError를 던진다.
    void openServer();

    // 타임아웃과 정상 연결 종료는 내부에서 기다린다.
    // 연결 수립 실패는 ConnectionError, 수신 및 메시지 처리 실패는
    // ReceiveError로 전달한다.
    std::optional<IncomingDto> receiveData();

    void sendBeamCommand(
        const beam::TransmitBeamCommand& command
    );

    void sendTrackEvents(
        const std::vector<target::TrackEvent>& trackEvents
    );

    // 모의기에서 받은 레이더 자세를 전시기에 전달한다.
    void sendRadarAttitude(
        const beam::SetAttitudeCommand& command
    );

    // 다른 상대에 보낸 것과 같은 메시지를 모니터에 전달한다.
    // 모니터가 연결되어 있지 않으면 무시한다.
    void sendToMonitor(const beam::TransmitBeamCommand& command);
    void sendToMonitor(const std::vector<target::TrackEvent>& trackEvents);
    void sendToMonitor(const beam::SetAttitudeCommand& command);
    // 모의기에서 받은 탐지 결과를 탐지 정보 메시지(0xBA01)로 보낸다.
    void sendToMonitor(const target::DetectionEvent& detectionEvent);
    // 요청 값에 현재 상대 연결 상태를 더해 모니터 상태 메시지를 보낸다.
    void sendToMonitor(const MonitorStatusRequest& request);

    // void requestStop() noexcept;
    // void closeServer() noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace comm
