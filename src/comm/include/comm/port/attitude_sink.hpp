#pragma once

#include <attitude/dto/attitude_dto.hpp>

namespace comm::port
{

class AttitudeSink
{
public:
    virtual ~AttitudeSink() = default;

    virtual void write(const attitude::AttitudeDto& attitude) = 0;
};

} // namespace comm::port
