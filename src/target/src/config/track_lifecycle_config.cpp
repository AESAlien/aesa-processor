#include <target/config/track_lifecycle_config.hpp>

#include <stdexcept>

namespace target
{

TrackLifecycleConfig::TrackLifecycleConfig(
    std::chrono::milliseconds confirmationDelay,
    std::chrono::milliseconds trackingInterval,
    std::uint32_t maxConsecutiveTrackingMisses
) : _confirmationDelay(confirmationDelay),
    _trackingInterval(trackingInterval),
    _maxConsecutiveTrackingMisses(maxConsecutiveTrackingMisses)
{
    if (_confirmationDelay.count() <= 0 || _trackingInterval.count() <= 0 || _maxConsecutiveTrackingMisses == 0)
    {
        throw std::invalid_argument("Invalid track lifecycle configuration");
    }
}

std::chrono::milliseconds TrackLifecycleConfig::confirmationDelay() const noexcept
{
    return _confirmationDelay;
}

std::chrono::milliseconds TrackLifecycleConfig::trackingInterval() const noexcept
{
    return _trackingInterval;
}

std::uint32_t TrackLifecycleConfig::maxConsecutiveTrackingMisses() const noexcept
{
    return _maxConsecutiveTrackingMisses;
}

std::chrono::milliseconds TrackLifecycleConfig::responseTimeout() const noexcept
{
    return _trackingInterval * _maxConsecutiveTrackingMisses;
}

TrackLifecycleConfig TrackLifecycleConfig::Builder::build() const
{
    if (!_confirmationDelay || !_trackingInterval || !_maxConsecutiveTrackingMisses)
    {
        throw std::logic_error("TrackLifecycleConfig::Builder requires every value");
    }
    return TrackLifecycleConfig(*_confirmationDelay, *_trackingInterval, *_maxConsecutiveTrackingMisses);
}

} // namespace target
