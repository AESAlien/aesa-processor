#pragma once

#include <target/config/association_config.hpp>
#include <target/config/measurement_config.hpp>
#include <target/config/tracking_config.hpp>
#include <target/config/track_management_config.hpp>

#include <optional>
#include <utility>

namespace target
{

struct TargetConfig
{
    class Builder
    {
    public:
        Builder& associationConfig(AssociationConfig value)
        {
            _association.emplace(std::move(value));
            return *this;
        }

        Builder& measurementConfig(MeasurementConfig value)
        {
            _measurement.emplace(std::move(value));
            return *this;
        }

        Builder& trackingConfig(TrackingConfig value)
        {
            _tracking.emplace(std::move(value));
            return *this;
        }

        Builder& trackManagementConfig(TrackManagementConfig value)
        {
            _trackManagement.emplace(std::move(value));
            return *this;
        }

        TargetConfig build() const;

    private:
        std::optional<AssociationConfig> _association;
        std::optional<MeasurementConfig> _measurement;
        std::optional<TrackingConfig> _tracking;
        std::optional<TrackManagementConfig> _trackManagement;
    };

    TargetConfig(
        AssociationConfig association,
        MeasurementConfig measurement,
        TrackingConfig tracking,
        TrackManagementConfig trackManagement
    );

    AssociationConfig association;
    MeasurementConfig measurement;
    TrackingConfig tracking;
    TrackManagementConfig trackManagement;
};

} // namespace target
