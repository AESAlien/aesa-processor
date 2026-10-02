#pragma once

#include <target/dto/track_event.hpp>

namespace comm::port
{

class TrackInformationSource
{
public:
    virtual ~TrackInformationSource() = default;

    virtual bool Read(target::TrackEvent& track) = 0;
};

} // namespace comm::port
