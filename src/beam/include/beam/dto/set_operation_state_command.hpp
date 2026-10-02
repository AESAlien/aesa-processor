#pragma once
#include <beam/domain/beam_status.hpp>

namespace beam
{

struct SetOperationStateCommand
{
    beam::OperationState powerState = beam::OperationState::OFF;
};

static_assert(sizeof(SetOperationStateCommand) == 1, "SetOperationStateCommand must be 1 bytes");

}   // namespace beam
