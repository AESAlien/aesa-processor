#pragma once

#include <beam/dto/control_dto.hpp>

namespace comm::port
{

class ControlSink
{
public:
    virtual ~ControlSink() = default;

    virtual void write(const beam::ControlDto& control) = 0;
};

} // namespace comm::port
