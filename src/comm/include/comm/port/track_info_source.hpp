#pragma once

#include <target/dto/track_snapshot_dto.hpp>

namespace comm::port
{

class TrackInformationSource
{
public:
    virtual ~TrackInformationSource() = default;

    virtual bool read(target::TrackSnapshotDto& track) = 0;
};

} // namespace comm::port
