#pragma once

#include <beam/dto/set_attitude_command.hpp>

namespace comm::port
{

class AttitudeSink
{
public:
    virtual ~AttitudeSink() = default;

    virtual void write(const beam::SetAttitudeCommand& command) = 0;
};

} // namespace comm::port
