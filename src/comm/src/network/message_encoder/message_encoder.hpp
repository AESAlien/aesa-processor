#pragma once

#include <beam/dto/beam_command.hpp>
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
    const beam::BeamCommand& command
);

EncodeResult encodeTrackEvents(
    const std::vector<target::TrackEvent>& trackEvents
);

} // namespace comm
