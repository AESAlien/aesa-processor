#pragma once

#include <beam/dto/attitude_dto.hpp>

namespace comm::port
{

class AttitudeSink
{
public:
    virtual ~AttitudeSink() = default;

    virtual void write(const beam::AttitudeDto& attitude) = 0;
};

} // namespace comm::port
