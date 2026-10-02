#pragma once

#include <beam/dto/set_operation_state_command.hpp>

namespace comm::port
{

class ControlSink
{
public:
    virtual ~ControlSink() = default;

    virtual void Write(const beam::SetOperationStateCommand& control) = 0;
};

} // namespace comm::port
