#pragma once
#include <beam/domain/beam_status.hpp>
#include <beam/dto/beam_command.hpp>
#include <cstdint>


namespace beam
{

class BeamCommandSource
{
public:
    virtual ~BeamCommandSource() = default;

    virtual bool read(BeamCommand& out) = 0;
};

}   // namespace beam