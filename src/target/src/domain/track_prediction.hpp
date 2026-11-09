#pragma once

#include <math/geometry/spherical_position.hpp>

#include <chrono>

namespace target
{

class TrackPrediction
{
public:
    TrackPrediction(
        std::chrono::milliseconds timestamp,
        math::SphericalPosition sphericalPosition
    );

    std::chrono::milliseconds timestamp() const noexcept;
    const math::SphericalPosition& sphericalPosition() const noexcept;

private:
    std::chrono::milliseconds _timestamp;
    math::SphericalPosition _sphericalPosition;
    void validate() const;
};

} // namespace target
