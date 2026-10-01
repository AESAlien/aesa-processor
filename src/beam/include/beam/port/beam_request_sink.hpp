#pragma once
#include <beam/domain/beam_status.hpp>
#include <beam/dto/beam_dto.hpp>
#include <cstdint>


namespace beam
{

class BeamRequestSink
{
public:
    virtual ~BeamRequestSink() = default;

    virtual bool write(const BeamDto& in) = 0;
};

}   // namespace beam