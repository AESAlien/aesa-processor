#pragma once

#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/linear_algebra/vector.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>
#include <cstddef>
#include <optional>

namespace target
{

struct TargetPrediction
{
    static constexpr std::size_t POSITION_DIMENSION = 3;

    TargetPrediction(
        std::chrono::milliseconds timestamp,
        math::SphericalPosition sphericalPosition,
        std::optional<math::SquareMatrix> positionCovariance = std::nullopt);

    const std::chrono::milliseconds timestamp;
    const math::SphericalPosition sphericalPosition;
    // Predicted Cartesian position covariance [px, py, pz] in m^2.
    // Absence is retained for algorithms that do not estimate uncertainty.
    // The producing algorithm must ensure positive semidefiniteness.
    const std::optional<math::SquareMatrix> positionCovariance;

private:
    void validate() const;
};

struct TargetTrackingState
{
    static constexpr std::size_t ACCELERATION_DIMENSION = 3;
    static constexpr std::size_t STATE_DIMENSION = 9;

    TargetTrackingState(
        math::SphericalPosition sphericalPosition,
        math::LinearVelocity dopplerVelocity,
        float power_db,
        std::chrono::milliseconds timestamp,
        math::CartesianPosition position,
        math::CartesianVelocity velocity,
        math::Vector acceleration = math::Vector(ACCELERATION_DIMENSION),
        std::optional<math::SquareMatrix> errorCovariance = std::nullopt);

    const math::SphericalPosition sphericalPosition;
    const math::LinearVelocity dopplerVelocity;
    const float power_db;
    const std::chrono::milliseconds timestamp;
    // Match math geometry: y at azimuth 0, x at azimuth +90, z upward.
    const math::CartesianPosition position;
    const math::CartesianVelocity velocity;
    // [ax, ay, az] in m/s^2 in the same antenna-fixed frame as position and velocity.
    const math::Vector acceleration;
    // SI state order: [px, py, pz, vx, vy, vz, ax, ay, az].
    // Each entry has the product of its row/column state units (m, m/s, m/s^2).
    // Absence means the algorithm has not estimated covariance, not zero uncertainty.
    // Construction checks shape, finite symmetry and nonnegative diagonal only;
    // the producing algorithm must ensure positive semidefiniteness.
    const std::optional<math::SquareMatrix> errorCovariance;

private:
    void validate() const;
};

} // namespace target
