#pragma once
#include <beam/domain/operation_state.hpp>

namespace beam
{

struct SetOperationStateCommand
{
    explicit SetOperationStateCommand(OperationState powerState)
      : powerState(powerState)
    {
    }

    const OperationState powerState;
};

static_assert(sizeof(SetOperationStateCommand) == 1, "SetOperationStateCommand must be 1 bytes");

} // namespace beam
