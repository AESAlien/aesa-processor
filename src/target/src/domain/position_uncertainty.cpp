#include "position_uncertainty.hpp"

#include "confirmed_track.hpp"
#include "tracking_algorithm.hpp"

#include <stdexcept>
#include <utility>

namespace target
{
namespace
{

void validatePositionCovariance(const std::optional<math::SquareMatrix>& covariance)
{
    if (!covariance)
    {
        return;
    }
    if (covariance->size() != 3)
    {
        throw std::invalid_argument("Position covariance requires a 3x3 matrix");
    }
    try
    {
        static_cast<void>(math::cholesky(*covariance));
    }
    catch (const std::domain_error&)
    {
        throw std::invalid_argument("Position covariance must be positive definite");
    }
}

} // namespace

std::optional<TrackPrediction> IPositionUncertainty::predict(
    const ConfirmedTrack& track,
    std::chrono::milliseconds time,
    std::optional<math::SquareMatrix>& covariance
) {
    covariance.reset();
    const auto* provider = dynamic_cast<const IPositionUncertainty*>(track._trackingAlgorithm.get());
    if (!provider)
    {
        return track.predict(time);
    }
    if (time < track.lastObservationTime())
    {
        return std::nullopt;
    }

    std::optional<math::SquareMatrix> predictedCovariance;
    auto prediction = ConfirmedTrack::validatePrediction(
        provider->predictWithPositionCovariance(track, time, predictedCovariance), time
    );
    if (prediction)
    {
        validatePositionCovariance(predictedCovariance);
        covariance = std::move(predictedCovariance);
    }
    return prediction;
}

std::optional<math::SquareMatrix> IPositionUncertainty::atLastObservation(const ConfirmedTrack& track)
{
    const auto* provider = dynamic_cast<const IPositionUncertainty*>(track._trackingAlgorithm.get());
    if (!provider)
    {
        return std::nullopt;
    }
    auto covariance = provider->observedPositionCovariance(track);
    validatePositionCovariance(covariance);
    return covariance;
}

} // namespace target
