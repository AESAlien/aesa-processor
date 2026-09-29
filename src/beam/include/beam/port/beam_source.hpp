#pragma once
#include <beam/domain/beam_status.hpp>
#include <beam/dto/BeamDto.hpp>
#include <chrono>
#include <cstdint>


namespace beam
{

class BeamSource
{
public:
    virtual ~BeamSource() = default;

    virtual BeamQueStatus read(dto::BeamDto& out, 
        std::chrono::milliseconds timeout) = 0;
};

}   // namespace beam