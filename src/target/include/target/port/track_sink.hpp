#pragma once

#include <target/dto/track.hpp>

namespace target {

class TrackSink
{
public:
    virtual ~TrackSink() = 0;

    virtual void write(const Track& track) = 0;
};

inline TrackSink::~TrackSink() = default;

} // namespace target
