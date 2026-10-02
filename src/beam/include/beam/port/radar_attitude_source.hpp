#pragma once
#include <beam/domain/radar_attitude.hpp>

namespace beam
{

class RadarAttitudeSource
{
public:
    virtual ~RadarAttitudeSource() = default;

    virtual bool Read(RadarAttitude& out) = 0;
};

} // namespace beam