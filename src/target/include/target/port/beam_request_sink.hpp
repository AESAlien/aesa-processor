#pragma once

#include <beam/dto/beam_request.hpp>

namespace target
{

class BeamRequestSink
{
public:
    virtual ~BeamRequestSink() = 0;

    virtual void write(const beam::BeamRequest& request) = 0;
};

inline BeamRequestSink::~BeamRequestSink() = default;

} // namespace target
