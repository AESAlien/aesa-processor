#pragma once

#include <pose/domain/enu_geo_transform.hpp>
#include <target/dto/track_event.hpp>
#include <target/config/track_reporting_config.hpp>

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
        TrackReportingConfig config
    );

    void setEnuGeoTransform(pose::EnuGeoTransform enuGeoTransform);
    std::vector<TrackEvent> getTrackEvents() const;

private:
    std::shared_ptr<const TrackRepository> _trackRepository;
    TrackReportingConfig _config;
    std::optional<pose::EnuGeoTransform> _enuGeoTransform;
};

} // namespace target
