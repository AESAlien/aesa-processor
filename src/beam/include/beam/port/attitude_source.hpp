#pragma once
#include <beam/dto/attitude_dto.hpp>

namespace beam
{

class AttitudePort
{
public:
    virtual ~AttitudePort() = default;
    
    virtual bool read(AttitudeDto& out) = 0;
};

}   // namespace beam