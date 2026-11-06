#pragma once

#include "algorithm/nds_algorithm.hpp"

#include <target/config/association_config.hpp>
#include <target/config/measurement_config.hpp>

#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/square_matrix.hpp>

#include <chrono>
#include <optional>

namespace target
{

class Observation;
class TentativeTrack;
class Track;

// Owns one prediction and factorization for a single observation batch.
class TrackingAssociation
{
public:
    std::optional<double> matches(const Observation& observation) const;

private:
    friend class DetectionMatcher;

    TrackingAssociation(
        std::chrono::milliseconds time,
        math::CartesianPosition position,
        NdsAlgorithm ndsAlgorithm,
        double ndsThreshold
    );

    std::chrono::milliseconds _time;
    math::CartesianPosition _position;
    NdsAlgorithm _ndsAlgorithm;
    double _ndsThreshold;
};

// Shared association checks and NDS distance scoring.
class DetectionMatcher
{
public:
    DetectionMatcher(AssociationConfig config, const MeasurementConfig& measurementConfig);
    // Return an NDS score inside the search duplicate gate; nullopt means unassociated.
    // Tentative scores use the residual beyond maxAssociationSpeed * elapsed time.
    // Tracking scores use predicted position covariance + measurement covariance.
    // Older searches use the stored position covariance with the motion allowance.
    std::optional<double> matchesPosition(const Observation& observation, const TentativeTrack& other) const;
    std::optional<double> matchesPosition(const Observation& observation, const Track& other) const;
    // Return an NDS rank after the speed/Doppler checks; nullopt means ineligible.
    // Confirmation ranking has no additional NDS threshold.
    std::optional<double> matchesConfirmation(const Observation& observation, const TentativeTrack& tentative) const;
    // Return an NDS score inside the position gate around the prediction at observation time.
    // Algorithms without predicted covariance retain the legacy two-measurement approximation.
    std::optional<double> matchesTracking(const Observation& observation, const Track& tracking) const;
    // Reuse only within one batch, before the track changes; retains no track reference.
    std::optional<TrackingAssociation> prepareTracking(
        const Track& tracking,
        std::chrono::milliseconds time
    ) const;

private:
    std::optional<double> positionScore(
        const Observation& observation,
        const math::SphericalPosition& referencePosition,
        std::chrono::milliseconds referenceTime,
        const math::SquareMatrix& covariance,
        bool compensateMotion
    ) const;

    const AssociationConfig _config;
    const math::SquareMatrix _measurementCovariance;
    const math::SquareMatrix _measurementDifferenceCovariance;
};

} // namespace target
