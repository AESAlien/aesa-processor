#pragma once

#include <beam/domain/operation_state.hpp>

namespace beam
{

struct SetOperationStateCommand
{
    const OperationState powerState;

    explicit SetOperationStateCommand(OperationState powerState)
      : powerState(powerState)
    {
    }
};

static_assert(sizeof(SetOperationStateCommand) == 1, "SetOperationStateCommand must be 1 bytes");

} // namespace beam
