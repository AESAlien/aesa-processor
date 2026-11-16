#pragma once

#include <pose/dto/set_radar_pose_command.hpp>
#include <beam/dto/transmit_beam_command.hpp>
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
    const pose::SetRadarPoseCommand& command
);

} // namespace comm
