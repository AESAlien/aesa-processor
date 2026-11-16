#pragma once

#include <pose/domain/enu_geo_transform.hpp>
#include <target/dto/track_event.hpp>

#include <cstddef>
#include <memory>
#include <optional>
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

    void setEnuGeoTransform(pose::EnuGeoTransform enuGeoTransform);
    std::vector<TrackEvent> getTrackEvents() const;

private:
    std::shared_ptr<const TrackRepository> _trackRepository;
    std::size_t _maxReportedTrackCount;
    std::optional<pose::EnuGeoTransform> _enuGeoTransform;
};

} // namespace target
