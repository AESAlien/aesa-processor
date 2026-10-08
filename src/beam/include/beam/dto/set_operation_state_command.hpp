#pragma once

#include <beam/domain/operation_state.hpp>

#include <optional>

namespace beam
{

struct SetOperationStateCommand
{
    const OperationState powerState;

    explicit SetOperationStateCommand(OperationState powerState)
      : powerState(powerState)
    {
    }

    class Builder
    {
    public:
        Builder& powerState(OperationState value) { _powerState = value; return *this; }

        SetOperationStateCommand build() const
        {
            return SetOperationStateCommand(_powerState.value());
        }

    private:
        std::optional<OperationState> _powerState;
    };
};

static_assert(sizeof(SetOperationStateCommand) == 1, "SetOperationStateCommand must be 1 bytes");

} // namespace beam
