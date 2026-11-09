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

namespace target
{

using math::literals::operator""_mps2;

class TargetPrediction
{
public:
    static constexpr std::size_t POSITION_DIMENSION = 3;

    TargetPrediction(
        std::chrono::milliseconds timestamp,
        math::SphericalPosition sphericalPosition,
        std::optional<math::SquareMatrix> positionCovariance = std::nullopt);

    std::chrono::milliseconds timestamp() const noexcept;
    const math::SphericalPosition& sphericalPosition() const noexcept;
    // Predicted Cartesian position covariance [px, py, pz] in m^2.
    // Absence is retained for algorithms that do not estimate uncertainty.
    // The producing algorithm must ensure positive semidefiniteness.
    const std::optional<math::SquareMatrix>& positionCovariance() const noexcept;

private:
    std::chrono::milliseconds _timestamp;
    math::SphericalPosition _sphericalPosition;
    std::optional<math::SquareMatrix> _positionCovariance;
    void validate() const;
};

class TargetTrackingState
{
public:
    static constexpr std::size_t STATE_DIMENSION = 9;

    TargetTrackingState(
        math::SphericalPosition sphericalPosition,
        math::LinearVelocity dopplerVelocity,
        float power_db,
        std::chrono::milliseconds timestamp,
        math::CartesianPosition position,
        math::CartesianVelocity velocity,
        math::CartesianAcceleration acceleration = math::CartesianAcceleration(
            0_mps2,
            0_mps2,
            0_mps2),
        std::optional<math::SquareMatrix> errorCovariance = std::nullopt);

    const math::SphericalPosition& sphericalPosition() const noexcept;
    const math::LinearVelocity& dopplerVelocity() const noexcept;
    float power_db() const noexcept;
    std::chrono::milliseconds timestamp() const noexcept;
    // Match math geometry: y at azimuth 0, x at azimuth +90, z upward.
    const math::CartesianPosition& position() const noexcept;
    const math::CartesianVelocity& velocity() const noexcept;
    // [ax, ay, az] in m/s^2 in the same antenna-fixed frame as position and velocity.
    const math::CartesianAcceleration& acceleration() const noexcept;
    // SI state order: [px, py, pz, vx, vy, vz, ax, ay, az].
    // Each entry has the product of its row/column state units (m, m/s, m/s^2).
    // Absence means the algorithm has not estimated covariance, not zero uncertainty.
    // Construction checks shape, finite symmetry and nonnegative diagonal only;
    // the producing algorithm must ensure positive semidefiniteness.
    const std::optional<math::SquareMatrix>& errorCovariance() const noexcept;

private:
    math::SphericalPosition _sphericalPosition;
    math::LinearVelocity _dopplerVelocity;
    float _power_db;
    std::chrono::milliseconds _timestamp;
    math::CartesianPosition _position;
    math::CartesianVelocity _velocity;
    math::CartesianAcceleration _acceleration;
    std::optional<math::SquareMatrix> _errorCovariance;
    void validate() const;
};

} // namespace target
