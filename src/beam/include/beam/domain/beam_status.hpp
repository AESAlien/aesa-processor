#pragma once
#include <cstdint>

namespace beam
{

enum class OperationState : bool
{
    On = true,
    Off = false
};

enum class BeamType : std::uint8_t
{
    Search = 1,
    Confirmation = 2,
    Tracking = 3
};

} // namespace beam
