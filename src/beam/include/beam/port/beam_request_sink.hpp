#pragma once
#include <beam/domain/beam_status.hpp>
#include <beam/dto/BeamDto.hpp>
#include <chrono>
#include <cstdint>


namespace beam
{

class BeamRequestSink
{
public:
    virtual ~BeamRequestSink() = default;

    virtual BeamQueStatus write(const dto::BeamDto& in) = 0;
};

}   // namespace beam