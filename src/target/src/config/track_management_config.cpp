#include <target/config/track_management_config.hpp>

#include <stdexcept>

namespace target
{

TrackManagementConfig::TrackManagementConfig(
    std::chrono::milliseconds confirmationDelay,
    std::chrono::milliseconds trackingInterval,
    std::uint32_t maxConsecutiveTrackingMisses,
    std::size_t maxReportedTrackCount
) : _confirmationDelay(confirmationDelay),
    _trackingInterval(trackingInterval),
    _maxConsecutiveTrackingMisses(maxConsecutiveTrackingMisses),
    _maxReportedTrackCount(maxReportedTrackCount)
{
    validate();
}

std::chrono::milliseconds TrackManagementConfig::confirmationDelay() const noexcept
{
    return _confirmationDelay;
}

std::chrono::milliseconds TrackManagementConfig::trackingInterval() const noexcept
{
    return _trackingInterval;
}

std::uint32_t TrackManagementConfig::maxConsecutiveTrackingMisses() const noexcept
{
    return _maxConsecutiveTrackingMisses;
}

std::size_t TrackManagementConfig::maxReportedTrackCount() const noexcept
{
    return _maxReportedTrackCount;
}

void TrackManagementConfig::validate() const
{
    if (_confirmationDelay.count() <= 0 || _trackingInterval.count() <= 0 ||
        _maxConsecutiveTrackingMisses == 0 || _maxReportedTrackCount == 0 ||
        _maxReportedTrackCount > MAXIMUM_REPORTED_TRACK_COUNT)
    {
        throw std::invalid_argument("Invalid track management configuration");
    }
}

TrackManagementConfig TrackManagementConfig::Builder::build() const
{
    if (!_confirmationDelay || !_trackingInterval || !_maxConsecutiveTrackingMisses || !_maxReportedTrackCount)
    {
        throw std::logic_error("TrackManagementConfig::Builder requires every value");
    }

    return TrackManagementConfig(
        *_confirmationDelay,
        *_trackingInterval,
        *_maxConsecutiveTrackingMisses,
        *_maxReportedTrackCount);
}

} // namespace target
