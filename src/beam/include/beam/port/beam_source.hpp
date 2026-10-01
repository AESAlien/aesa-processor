#pragma once
#include <beam/domain/beam_status.hpp>
#include <beam/dto/beam_dto.hpp>
#include <cstdint>


namespace beam
{

class BeamSource
{
public:
    virtual ~BeamSource() = default;

    virtual bool read(BeamDto& out) = 0;
};

}   // namespace beam