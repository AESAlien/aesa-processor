#pragma once
#include <beam/domain/beam_status.hpp>
#include <beam/dto/beam_request.hpp>
#include <cstdint>

namespace beam
{

class BeamRequestSink
{
public:
    virtual ~BeamRequestSink() = default;

    virtual bool Write(const BeamRequest& in) = 0;
};

} // namespace beam