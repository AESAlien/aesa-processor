#include <target/config/target_config.hpp>

#include <stdexcept>
#include <utility>

namespace target
{

TargetConfig::TargetConfig(
    AssociationConfig association,
    MeasurementConfig measurement,
    TrackingConfig tracking,
    TrackLifecycleConfig trackLifecycle,
    TrackReportingConfig trackReporting
) : association(std::move(association)),
    measurement(std::move(measurement)),
    tracking(std::move(tracking)),
    trackLifecycle(std::move(trackLifecycle)),
    trackReporting(std::move(trackReporting))
{
}

TargetConfig TargetConfig::Builder::build() const
{
    if (!_association || !_measurement || !_tracking || !_trackLifecycle || !_trackReporting)
    {
        throw std::logic_error("TargetConfig::Builder requires every value");
    }

    return TargetConfig(
        *_association,
        *_measurement,
        *_tracking,
        *_trackLifecycle,
        *_trackReporting
    );
}

} // namespace target
