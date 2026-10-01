#pragma once
#include <beam/domain/beam_status.hpp>

namespace beam
{

struct ControlDto
{
    beam::ControlType powerState = beam::ControlType::OFF;
};

static_assert(sizeof(ControlDto) == 1, "ControlDto must be 1 bytes");

}   // namespace beam
