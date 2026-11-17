#include "operation/operation_state_gate.hpp"

namespace app
{
bool OperationStateGate::apply(beam::OperationState state)
{
    if (state == _state)
    {
        return false;
    }

    _state = state;
    return true;
}

bool OperationStateGate::isOn() const
{
    return _state == beam::OperationState::ON;
}

} // namespace app
