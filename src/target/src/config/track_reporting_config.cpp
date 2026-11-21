#include <target/config/track_reporting_config.hpp>

#include <stdexcept>

namespace target
{

TrackReportingConfig::TrackReportingConfig(std::size_t maxReportedTrackCount)
  : _maxReportedTrackCount(maxReportedTrackCount)
{
    if (_maxReportedTrackCount == 0 || _maxReportedTrackCount > MAXIMUM_REPORTED_TRACK_COUNT)
    {
        throw std::invalid_argument("Invalid track reporting configuration");
    }
}

std::size_t TrackReportingConfig::maxReportedTrackCount() const noexcept
{
    return _maxReportedTrackCount;
}

TrackReportingConfig TrackReportingConfig::Builder::build() const
{
    if (!_maxReportedTrackCount)
    {
        throw std::logic_error("TrackReportingConfig::Builder requires every value");
    }
    return TrackReportingConfig(*_maxReportedTrackCount);
}

} // namespace target
