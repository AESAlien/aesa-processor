#include "track_prediction.hpp"

#include <stdexcept>
#include <utility>

namespace target
{

TrackPrediction::TrackPrediction(
    std::chrono::milliseconds timestamp,
    math::SphericalPosition sphericalPosition
) : _timestamp(timestamp),
    _sphericalPosition(std::move(sphericalPosition))
{
    validate();
}

std::chrono::milliseconds TrackPrediction::timestamp() const noexcept
{
    return _timestamp;
}

const math::SphericalPosition& TrackPrediction::sphericalPosition() const noexcept
{
    return _sphericalPosition;
}

void TrackPrediction::validate() const
{
    if (_timestamp.count() < 0 || _sphericalPosition.radius.isNonpositive())
    {
        throw std::invalid_argument("Prediction requires a positive range and nonnegative time");
    }
}

} // namespace target
