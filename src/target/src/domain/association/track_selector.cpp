#include "track_selector.hpp"

#include "domain/tentative_track.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/quantity/quantity_tolerance.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace target
{

namespace
{

template <typename TargetType, typename Prepare, typename Match>
std::optional<Observation> selectBestObservation(
    const TargetType& target,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations,
    const Prepare& prepare,
    const Match& match
) {
    if (resultTime <= target.lastResultTime())
    {
        return std::nullopt;
    }

    const auto first = std::find_if(
        observations.begin(),
        observations.end(),
        [resultTime](const Observation& observation)
        {
            return observation.observationTime() == resultTime;
        }
    );
    if (first == observations.end())
    {
        return std::nullopt;
    }
    // 유효한 시각의 관측이 있을 때만 한 번 준비하고 모든 후보에 재사용한다.
    const auto calculation = prepare();
    if (!calculation)
    {
        return std::nullopt;
    }

    const Observation* bestObservation = nullptr;
    auto bestScore = 0.0;
    for (auto current = first; current != observations.end(); ++current)
    {
        const auto& observation = *current;
        if (observation.observationTime() != resultTime)
        {
            continue;
        }

        const auto score = match(observation, *calculation);
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
    std::unique_ptr<const IAssociationAlgorithm> algorithm
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
    return selectBestObservation(
        confirmedTrack,
        resultTime,
        observations,
        [&]()
        {
            return _algorithm->predictAndPrepare(confirmedTrack, resultTime);
        },
        [&](const Observation& observation, const AssociationPrediction& prepared)
        {
            const auto& [prediction, calculation] = prepared;
            const auto score = calculation->calculate(observation.sphericalPosition(), prediction.sphericalPosition());
            return score && *score <= _config.positionNdsThreshold() ? score : std::nullopt;
        }
    );
}

std::optional<Observation> TrackSelector::selectConfirmationObservation(
    const TentativeTrack& tentative,
    std::chrono::milliseconds resultTime,
    const std::vector<Observation>& observations
) const {
    return selectBestObservation(
        tentative,
        resultTime,
        observations,
        [&]()
        {
            return _algorithm->prepareMeasurements();
        },
        [&](const Observation& observation, const IAssociationCalculation& calculation)
        {
            return confirmationScore(calculation, observation, tentative);
        }
    );
}

std::optional<double> TrackSelector::confirmationScore(
    const IAssociationCalculation& calculation,
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

        return calculation.calculate(observation.sphericalPosition(), tentative.observation().sphericalPosition());
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

} // namespace target
