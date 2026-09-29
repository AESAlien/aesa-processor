#pragma once

#include <beam/dto/beam_dto.hpp>

namespace comm::port
{

class BeamCommandSource
{
public:
    virtual ~BeamCommandSource() = default;

    virtual bool read(beam::BeamDto& command) = 0;
};

} // namespace comm::port
