#pragma once

#include <math/bounded/distance.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/vector.hpp>

#include <optional>

namespace target
{

class TargetPrediction;
class TrackState;

// Owns the current prepared state. Each selector must own an independent instance.
// Preparation replaces that state; failure clears it. Do not interleave batches
// or access one instance concurrently. calculate does not alter prepared state.
class IAssociationAlgorithm
{
public:
    virtual ~IAssociationAlgorithm() = default;

    virtual bool prepareMeasurements() = 0;
    virtual bool preparePrediction(const TargetPrediction& prediction) = 0;
    virtual bool prepareState(const TrackState& state) = 0;

    // Lower scores are better; nullopt means unprepared or an unusable calculation.
    virtual std::optional<double> calculate(const math::Vector& residual) const = 0;
    virtual std::optional<double> calculate(
        const math::SphericalPosition& observation,
        const math::SphericalPosition& reference,
        std::optional<math::Distance> maximumTravel = std::nullopt
    ) const = 0;
};

} // namespace target
