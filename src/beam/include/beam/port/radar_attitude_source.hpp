#pragma once
#include <beam/domain/radar_attitude.hpp>

namespace beam
{

class RadarAttitudeSource
{
public:
    virtual ~RadarAttitudeSource() = default;
    
    virtual bool read(RadarAttitude& out) = 0;
};

}   // namespace beam