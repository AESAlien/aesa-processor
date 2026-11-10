#pragma once

#include <target/dto/track_event.hpp>

#include <cstddef>
#include <memory>
#include <vector>

namespace target
{

class TrackRepository;

class TrackReporter
{
public:
    TrackReporter(
        std::shared_ptr<const TrackRepository> repository,
        std::size_t maxReportedTrackCount
    );

    std::vector<TrackEvent> nextTrackEvents() const;

private:
    std::shared_ptr<const TrackRepository> _trackRepository;
    std::size_t _maxReportedTrackCount;
};

} // namespace target
