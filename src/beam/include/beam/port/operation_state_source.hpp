#pragma once
#include <beam/domain/operation_state.hpp>
#include <beam/dto/set_operation_state_command.hpp>
#include <cstdint>

namespace beam
{

class OperationStateSource
{
public:
    virtual ~OperationStateSource() = default;

    virtual bool read(SetOperationStateCommand& out) = 0;
};

} // namespace beam