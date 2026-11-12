#pragma once

#include <message/dto/monitor_status.hpp>

#include <beam/dto/set_attitude_command.hpp>
#include <beam/dto/transmit_beam_command.hpp>
#include <target/dto/detection_event.hpp>
#include <target/dto/track_event.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace comm
{

enum class EncodingError
{
    NONE,
    TOO_MANY_TRACKS,
    TOO_MANY_PLOTS,
    TYPE_CONVERSION_ERROR
};

struct EncodeResult
{
    std::optional<std::vector<uint8_t>> bytes;
    EncodingError error = EncodingError::NONE;
};

EncodeResult encodeBeamCommand(
    const beam::TransmitBeamCommand& command
);

EncodeResult encodeTrackEvents(
    const std::vector<target::TrackEvent>& trackEvents
);

// 모의기에서 받은 레이더 자세를 전시기용 자세 메시지로 인코딩한다.
EncodeResult encodeRadarAttitude(
    const beam::SetAttitudeCommand& command
);

// 모의기에서 받은 탐지 결과를 모니터용 탐지 정보 메시지로 인코딩한다.
// 빔 명령은 종류, 송신 시각, ID만 채운다. 방향과 빔 폭은 같은 시각과 ID의
// 빔 송신 명령에서 찾을 수 있으므로 0으로 둔다. DetectionEvent에 없는 플롯 ID도 0이다.
EncodeResult encodeDetectionEvent(
    const target::DetectionEvent& detectionEvent
);

// 데이터 처리부 상태를 모니터 상태 메시지로 인코딩한다.
EncodeResult encodeMonitorStatus(
    const MonitorStatus& status
);

} // namespace comm
