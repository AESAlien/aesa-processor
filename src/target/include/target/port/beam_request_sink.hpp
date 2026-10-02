#pragma once

#include <beam/dto/beam_dto.hpp>

namespace target
{

class BeamRequestSink
{
public:
    virtual ~BeamRequestSink() = 0;

    virtual void Write(const beam::BeamDto& request) = 0;
};

inline BeamRequestSink::~BeamRequestSink() = default;

} // namespace target
