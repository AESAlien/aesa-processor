#pragma once
#include <cstdint>

namespace beam
{
    enum class ControlType : bool { ON = true, OFF = false };
    enum class BeamType : std::uint8_t { Search = 1, Confirmation = 2, Tracking = 3 };
    enum class BeamQueStatus : std::uint8_t { Success, Timeout, Closed };
}
