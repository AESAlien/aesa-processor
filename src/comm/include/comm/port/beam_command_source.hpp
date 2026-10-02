#pragma once

#include <beam/dto/beam_command.hpp>

namespace comm::port
{

class BeamCommandSource
{
public:
    virtual ~BeamCommandSource() = default;

    virtual bool read(beam::BeamCommand& command) = 0;
};

} // namespace comm::port
