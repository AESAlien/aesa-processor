#pragma once

#include <target/dto/track_snapshot.hpp>

namespace target {

class TrackSink
{
public:
    virtual ~TrackSink() = 0;

    virtual void write(const TrackSnapshot& snapshot) = 0;
};

inline TrackSink::~TrackSink() = default;

} // namespace target
