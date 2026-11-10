#pragma once

#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_acceleration.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>
#include <cstddef>

namespace target
{

using math::literals::operator""_mps2;

class TrackState
{
public:
    static constexpr std::size_t STATE_DIMENSION = 9;

    TrackState(
        math::SphericalPosition sphericalPosition,
        math::LinearVelocity dopplerVelocity,
        float power_db,
        std::chrono::milliseconds timestamp,
        math::CartesianPosition position,
        math::CartesianVelocity velocity,
        math::CartesianAcceleration acceleration = math::CartesianAcceleration(
            0_mps2,
            0_mps2,
            0_mps2
        )
    );

    const math::SphericalPosition& sphericalPosition() const noexcept;
    const math::LinearVelocity& dopplerVelocity() const noexcept;
    float power_db() const noexcept;
    std::chrono::milliseconds timestamp() const noexcept;
    // 위치, 속도, 가속도는 안테나 고정 좌표계의 성분이다.
    // 방위각 0°는 y축, +90°는 x축, 위쪽은 z축이다.
    const math::CartesianPosition& position() const noexcept;
    const math::CartesianVelocity& velocity() const noexcept;
    const math::CartesianAcceleration& acceleration() const noexcept;

private:
    math::SphericalPosition _sphericalPosition;
    math::LinearVelocity _dopplerVelocity;
    float _power_db;
    std::chrono::milliseconds _timestamp;
    math::CartesianPosition _position;
    math::CartesianVelocity _velocity;
    math::CartesianAcceleration _acceleration;
    void validate() const;
};

} // namespace target
