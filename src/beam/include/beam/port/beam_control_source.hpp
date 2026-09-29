#pragma once
#include <beam/domain/beam_status.hpp>
#include <beam/dto/ControlDto.hpp>
#include <chrono>
#include <cstdint>


namespace beam
{

class BeamControlSource
{
public:
    virtual ~BeamControlSource() = default;

    virtual BeamQueStatus read(dto::ControlDto& out , 
        std::chrono::milliseconds timeout) = 0;
};

}   // namespace beam