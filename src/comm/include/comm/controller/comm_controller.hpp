#pragma once

#include <beam/dto/beam_command.hpp>
#include <comm/controller/comm_error.hpp>
#include <comm/dto/incoming_dto.hpp>
#include <target/dto/track_event.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace comm
{

struct CommEndpoint
{
    std::string address;
    uint16_t port = 0;
};

struct CommConfiguration
{
    CommEndpoint server;
    CommEndpoint operatorConsole;
    CommEndpoint scenarioSimulator;
};

class CommController
{
public:
    explicit CommController(CommConfiguration configuration);
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
        const beam::BeamCommand& command
    );

    void sendTrackEvents(
        const std::vector<target::TrackEvent>& trackEvents
    );

    // void requestStop() noexcept;
    // void closeServer() noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace comm
