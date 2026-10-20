#pragma once

#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_acceleration.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>
#include <cstddef>
#include <optional>

namespace aesa_processor::target
{

using aesa_processor::math::literals::operator""_mps2;

struct TargetPrediction
{
    static constexpr std::size_t POSITION_DIMENSION = 3;

    TargetPrediction(
        std::chrono::milliseconds timestamp,
        aesa_processor::math::SphericalPosition sphericalPosition,
        std::optional<aesa_processor::math::SquareMatrix> positionCovariance = std::nullopt);

    const std::chrono::milliseconds timestamp;
    const aesa_processor::math::SphericalPosition sphericalPosition;
    // Predicted Cartesian position covariance [px, py, pz] in m^2.
    // Absence is retained for algorithms that do not estimate uncertainty.
    // The producing algorithm must ensure positive semidefiniteness.
    const std::optional<aesa_processor::math::SquareMatrix> positionCovariance;

private:
    void validate() const;
};

struct TargetTrackingState
{
    static constexpr std::size_t STATE_DIMENSION = 9;

    TargetTrackingState(
        aesa_processor::math::SphericalPosition sphericalPosition,
        aesa_processor::math::LinearVelocity dopplerVelocity,
        float power_db,
        std::chrono::milliseconds timestamp,
        aesa_processor::math::CartesianPosition position,
        aesa_processor::math::CartesianVelocity velocity,
        aesa_processor::math::CartesianAcceleration acceleration = aesa_processor::math::CartesianAcceleration(
            0_mps2,
            0_mps2,
            0_mps2),
        std::optional<aesa_processor::math::SquareMatrix> errorCovariance = std::nullopt);

    const aesa_processor::math::SphericalPosition sphericalPosition;
    const aesa_processor::math::LinearVelocity dopplerVelocity;
    const float power_db;
    const std::chrono::milliseconds timestamp;
    // Match math geometry: y at azimuth 0, x at azimuth +90, z upward.
    const aesa_processor::math::CartesianPosition position;
    const aesa_processor::math::CartesianVelocity velocity;
    // [ax, ay, az] in m/s^2 in the same antenna-fixed frame as position and velocity.
    const aesa_processor::math::CartesianAcceleration acceleration;
    // SI state order: [px, py, pz, vx, vy, vz, ax, ay, az].
    // Each entry has the product of its row/column state units (m, m/s, m/s^2).
    // Absence means the algorithm has not estimated covariance, not zero uncertainty.
    // Construction checks shape, finite symmetry and nonnegative diagonal only;
    // the producing algorithm must ensure positive semidefiniteness.
    const std::optional<aesa_processor::math::SquareMatrix> errorCovariance;

private:
    void validate() const;
};

} // namespace aesa_processor::target
