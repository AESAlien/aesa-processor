#pragma once
#include <beam/domain/beam_status.hpp>
#include <beam/dto/control_dto.hpp>
#include <cstdint>


namespace beam
{

class BeamControlSource
{
public:
    virtual ~BeamControlSource() = default;

    virtual bool read(ControlDto& out) = 0;
};

}   // namespace beam