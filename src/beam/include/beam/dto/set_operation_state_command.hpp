#pragma once
#include <beam/domain/operation_state.hpp>

namespace beam
{

struct SetOperationStateCommand
{
    OperationState powerState = OperationState::OFF;
};

static_assert(sizeof(SetOperationStateCommand) == 1, "SetOperationStateCommand must be 1 bytes");

} // namespace beam
