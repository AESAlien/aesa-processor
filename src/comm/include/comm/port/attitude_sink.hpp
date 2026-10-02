#pragma once

#include <beam/dto/radar_attitude.hpp>

namespace comm::port
{

class AttitudeSink
{
public:
    virtual ~AttitudeSink() = default;

    virtual void write(const beam::RadarAttitude& attitude) = 0;
};

} // namespace comm::port
