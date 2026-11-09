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

std::optional<std::size_t> TentativeSelector::selectNewCandidate(
    const std::vector<Observation>& observations,
    const std::list<Track>& tracks
) const {
    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        const Observation& observation = observations[index];
        bool associated = false;
        for (const auto& target : tracks)
        {
            const auto score = std::visit(
                [&](const auto& registeredTarget)
                {
                    return matchesPosition(observation, registeredTarget);
                },
                target
            );
            if (score)
            {
                associated = true;
                break;
            }
        }

        if (!associated)
        {
            return index;
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
        std::optional<math::Length> maximumTravel;
        if (compensateMotion)
        {
            const auto elapsed = std::chrono::abs(observation.lastObservationTime() - referenceTime);
            maximumTravel = _config.maxAssociationSpeed() * elapsed;
        }
        const auto score = _algorithm->calculate(observation.sphericalPosition(), referencePosition, maximumTravel);
        return score && *score <= _config.positionNdsThreshold() ? score : std::nullopt;
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

std::optional<double> TentativeSelector::matchesPosition(
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
        first.lastObservationTime(),
        true
    );
}

std::optional<double> TentativeSelector::matchesPosition(
    const Observation& observation,
    const ConfirmedTrack& other
) const {
    const auto time = observation.lastObservationTime();
    const auto referenceTime = other.lastObservationTime();
    if (time >= referenceTime)
    {
        const auto prediction = other.predict(time);
        if (!prediction)
        {
            return std::nullopt;
        }
        if (!_algorithm->preparePrediction(*prediction))
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

    return _algorithm->prepareState(*other.state())
        ? positionScore(observation, other.sphericalPosition(), referenceTime, true)
        : std::nullopt;
}

} // namespace target
