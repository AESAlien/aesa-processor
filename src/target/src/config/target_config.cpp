#include <target/config/target_config.hpp>

#include <stdexcept>
#include <utility>

namespace target
{

TargetConfig::TargetConfig(
    AssociationConfig association,
    MeasurementConfig measurement,
    TrackingConfig tracking,
    TrackManagementConfig trackManagement
) : association(std::move(association)),
    measurement(std::move(measurement)),
    tracking(std::move(tracking)),
    trackManagement(std::move(trackManagement))
{
}

TargetConfig TargetConfig::Builder::build() const
{
    if (!_association || !_measurement || !_tracking || !_trackManagement)
    {
        throw std::logic_error("TargetConfig::Builder requires every value");
    }

    return TargetConfig(
        *_association,
        *_measurement,
        *_tracking,
        *_trackManagement
    );
}

} // namespace target
