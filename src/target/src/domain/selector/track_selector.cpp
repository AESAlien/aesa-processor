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
std::optional<Observation> selectBestObservation(
    const TargetType& target,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations,
    const Match& match
) {
    if (resultTime <= target.lastResultTime())
    {
        return std::nullopt;
    }

    const Observation* bestObservation = nullptr;
    auto bestScore = 0.0;
    for (const auto& observation : observations)
    {
        if (observation.observationTime() != resultTime)
        {
            continue;
        }

        const auto score = match(observation, target);
        // 점수가 같으면 먼저 나온 관측을 유지한다.
        if (score && (!bestObservation || *score < bestScore))
        {
            bestObservation = &observation;
            bestScore = *score;
        }
    }
    if (bestObservation)
    {
        return *bestObservation;
    }
    return std::nullopt;
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

std::optional<Observation> TrackSelector::selectTrackingObservation(
    const ConfirmedTrack& confirmedTrack,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations
) const {
    auto prepared = false;
    std::optional<TrackPrediction> prediction;
    return selectBestObservation(
        confirmedTrack,
        resultTime,
        observations,
        [&](const Observation& observation, const ConfirmedTrack& current)
        {
            // 결과 시각에 맞는 관측이 처음 들어올 때만 예측한다.
            if (!prepared)
            {
                prepared = true;
                prediction = _algorithm->predictAndPrepare(current, resultTime);
            }
            if (!prediction)
            {
                return std::optional<double>();
            }
            const auto score = _algorithm->calculate(observation.sphericalPosition(), prediction->sphericalPosition());
            return score && *score <= _config.positionNdsThreshold() ? score : std::nullopt;
        }
    );
}

std::optional<Observation> TrackSelector::selectConfirmationObservation(
    const TentativeTrack& tentative,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations
) const {
    auto prepared = false;
    auto ready = false;
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
    const auto time = observation.observationTime();
    if (time <= tentative.lastResultTime())
    {
        return std::nullopt;
    }

    const auto position = observation.sphericalPosition().toCartesian();
    const auto initialPosition = tentative.observation().sphericalPosition().toCartesian();
    try
    {
        const auto displacement = position - initialPosition;
        const auto elapsed = std::chrono::duration<double>(time - tentative.lastResultTime());

        const auto speed = (displacement / elapsed).speed();
        if (speed > _config.maxAssociationSpeed())
        {
            return std::nullopt;
        }

        const auto dopplerTolerance = math::QuantityTolerance<math::LinearVelocity>(
            _config.maxInitialDopplerDifference().asLinearVelocity(),
            0.0
        );
        if (observation.dopplerVelocity().notEquals(tentative.observation().dopplerVelocity(), dopplerTolerance))
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
