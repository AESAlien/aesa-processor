#include "track_selector.hpp"

#include "domain/tentative_track.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/quantity/quantity_tolerance.hpp>

#include <stdexcept>
#include <utility>

namespace target
{

namespace
{

template <typename TargetType, typename Match>
std::optional<std::size_t> selectBestObservation(
    const TargetType& target,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations,
    const Match& match
) {
    if (resultTime <= target.lastResultTime())
    {
        return std::nullopt;
    }

    std::optional<std::size_t> bestIndex;
    double bestScore = 0.0;
    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        const auto& observation = observations[index];
        if (observation.lastObservationTime() != resultTime)
        {
            continue;
        }

        const auto score = match(observation, target);
        // Keep the first observation when two eligible scores are equal.
        if (score && (!bestIndex || *score < bestScore))
        {
            bestIndex = index;
            bestScore = *score;
        }
    }
    return bestIndex;
}

} // namespace

TrackSelector::TrackSelector(
    AssociationConfig config,
    std::unique_ptr<IAssociationAlgorithm> algorithm
) : _config(std::move(config)),
    _algorithm(std::move(algorithm))
{
    if (!_algorithm)
    {
        throw std::invalid_argument("Association algorithm must not be null");
    }
}

std::optional<std::size_t> TrackSelector::selectObservation(
    const ConfirmedTrack& track,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations
) const {
    bool prepared = false;
    std::optional<math::SphericalPosition> predictedPosition;
    bool ready = false;
    // Empty batches and observations at other times must not trigger prediction.
    return selectBestObservation(
        track,
        resultTime,
        observations,
        [&](const Observation& observation, const ConfirmedTrack& current)
        {
            if (!prepared)
            {
                prepared = true;
                const auto prediction = current.predict(resultTime);
                if (prediction)
                {
                    predictedPosition = prediction->sphericalPosition();
                    ready = _algorithm->preparePrediction(*prediction);
                }
            }
            if (!ready)
            {
                return std::optional<double>();
            }
            const auto score = _algorithm->calculate(observation.sphericalPosition(), *predictedPosition);
            return score && *score <= _config.positionNdsThreshold() ? score : std::nullopt;
        }
    );
}

std::optional<std::size_t> TrackSelector::selectConfirmation(
    const TentativeTrack& tentative,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations
) const {
    bool prepared = false;
    bool ready = false;
    return selectBestObservation(
        tentative,
        resultTime,
        observations,
        [&](const Observation& observation, const TentativeTrack& current)
        {
            if (!prepared)
            {
                prepared = true;
                ready = _algorithm->prepareMeasurements();
            }
            return ready ? confirmationScore(observation, current) : std::nullopt;
        }
    );
}

std::optional<double> TrackSelector::confirmationScore(
    const Observation& observation,
    const TentativeTrack& tentative
) const {
    const auto time = observation.lastObservationTime();
    if (time <= tentative.lastResultTime())
    {
        return std::nullopt;
    }

    const auto position = observation.sphericalPosition().toCartesian();
    const auto initialPosition = tentative.observation().sphericalPosition().toCartesian();
    try
    {
        const auto displacement = position - initialPosition;
        // Ordered nonnegative times keep this integer subtraction within its range.
        const auto elapsed = std::chrono::duration<double>(time - tentative.lastResultTime());
        const auto speed = displacement.toVelocity(elapsed).speed();
        if (speed.greaterThan(_config.maxAssociationSpeed(), math::LinearVelocity::NONE_TOLERANCE) ||
            !observation.dopplerVelocity().equals(
                tentative.observation().dopplerVelocity(),
                math::QuantityTolerance<math::LinearVelocity>(_config.maxInitialDopplerDifference(), 0.0)
            ))
        {
            return std::nullopt;
        }

        return _algorithm->calculate(observation.sphericalPosition(), tentative.observation().sphericalPosition());
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

} // namespace target
