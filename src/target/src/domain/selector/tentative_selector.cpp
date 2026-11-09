#include "tentative_selector.hpp"

#include "domain/tentative_track.hpp"

#include <stdexcept>
#include <utility>

namespace target
{

TentativeSelector::TentativeSelector(
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

std::optional<Observation> TentativeSelector::selectNewCandidate(
    const std::vector<Observation>& observations,
    const std::list<Track>& tracks
) const {
    for (const auto& observation : observations)
    {
        auto associated = false;
        for (const auto& track : tracks)
        {
            const auto score = std::visit(
                [&](const auto& existingTrack)
                {
                    return matchingPositionScore(observation, existingTrack);
                },
                track
            );
            if (score)
            {
                associated = true;
                break;
            }
        }

        if (!associated)
        {
            return observation;
        }
    }

    return std::nullopt;
}

std::optional<double> TentativeSelector::positionScore(
    const Observation& observation,
    const math::SphericalPosition& referencePosition,
    std::chrono::milliseconds referenceTime,
    bool compensateMotion
) const {
    try
    {
        std::optional<math::Distance> maximumTravel;
        if (compensateMotion)
        {
            const auto elapsed = std::chrono::abs(observation.observationTime() - referenceTime);
            maximumTravel = math::Distance(_config.maxAssociationSpeed().asLinearVelocity() * elapsed);
        }
        const auto score = _algorithm->calculate(observation.sphericalPosition(), referencePosition, maximumTravel);
        return score && *score <= _config.positionNdsThreshold() ? score : std::nullopt;
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

std::optional<double> TentativeSelector::matchingPositionScore(
    const Observation& observation,
    const TentativeTrack& other
) const {
    if (!_algorithm->prepareMeasurements())
    {
        return std::nullopt;
    }
    const auto& first = other.observation();
    return positionScore(
        observation,
        first.sphericalPosition(),
        first.observationTime(),
        true
    );
}

std::optional<double> TentativeSelector::matchingPositionScore(
    const Observation& observation,
    const ConfirmedTrack& other
) const {
    const auto time = observation.observationTime();
    const auto referenceTime = other.lastObservationTime();
    if (time >= referenceTime)
    {
        const auto prediction = _algorithm->predictAndPrepare(other, time);
        if (!prediction)
        {
            return std::nullopt;
        }
        return positionScore(
            observation,
            prediction->sphericalPosition(),
            referenceTime,
            false
        );
    }

    return _algorithm->prepareFromLastObservation(other)
        ? positionScore(observation, other.sphericalPosition(), referenceTime, true)
        : std::nullopt;
}

} // namespace target
