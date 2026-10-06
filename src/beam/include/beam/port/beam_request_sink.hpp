#pragma once
#include <beam/domain/operation_state.hpp>
#include <beam/dto/beam_request.hpp>
#include <cstdint>

namespace beam
{

class BeamRequestSink
{
public:
    virtual ~BeamRequestSink() = default;

    virtual bool write(const BeamRequest& in) = 0;
};

} // namespace beam