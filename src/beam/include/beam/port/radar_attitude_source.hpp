#pragma once
#include <beam/dto/set_attitude_command.hpp>

namespace beam
{

class RadarAttitudeSource
{
public:
    virtual ~RadarAttitudeSource() = default;

    virtual bool read(SetAttitudeCommand& out) = 0;
};

} // namespace beam