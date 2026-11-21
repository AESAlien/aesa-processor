#pragma once

#include <target/config/association_config.hpp>
#include <target/config/measurement_config.hpp>
#include <target/config/tracking_config.hpp>
#include <target/config/track_lifecycle_config.hpp>
#include <target/config/track_reporting_config.hpp>

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

        Builder& trackLifecycleConfig(TrackLifecycleConfig value)
        {
            _trackLifecycle.emplace(std::move(value));
            return *this;
        }

        Builder& trackReportingConfig(TrackReportingConfig value)
        {
            _trackReporting.emplace(std::move(value));
            return *this;
        }

        TargetConfig build() const;

    private:
        std::optional<AssociationConfig> _association;
        std::optional<MeasurementConfig> _measurement;
        std::optional<TrackingConfig> _tracking;
        std::optional<TrackLifecycleConfig> _trackLifecycle;
        std::optional<TrackReportingConfig> _trackReporting;
    };

    TargetConfig(
        AssociationConfig association,
        MeasurementConfig measurement,
        TrackingConfig tracking,
        TrackLifecycleConfig trackLifecycle,
        TrackReportingConfig trackReporting
    );

    AssociationConfig association;
    MeasurementConfig measurement;
    TrackingConfig tracking;
    TrackLifecycleConfig trackLifecycle;
    TrackReportingConfig trackReporting;
};

} // namespace target
