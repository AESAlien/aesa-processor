#pragma once

#include <target/dto/track_event.hpp>

namespace target
{

class TrackSink
{
public:
    virtual ~TrackSink() = 0;

    virtual void write(const TrackEvent& event) = 0;
};

inline TrackSink::~TrackSink() = default;

} // namespace target
