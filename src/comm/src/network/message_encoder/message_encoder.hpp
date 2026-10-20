#pragma once

#include <beam/dto/transmit_beam_command.hpp>
#include <target/dto/track_event.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace aesa_processor::comm
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
    const aesa_processor::beam::TransmitBeamCommand& command
);

EncodeResult encodeTrackEvents(
    const std::vector<aesa_processor::target::TrackEvent>& trackEvents
);

} // namespace aesa_processor::comm
