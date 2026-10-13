#pragma once

#include <math/quantity/angle.hpp>
#include <math/linear_algebra/square_matrix.hpp>

#include <chrono>
#include <cstddef>

namespace beam
{

using math::literals::operator""_deg;

struct AttitudeConfig
{
    math::Angle maxAbsPitch = 89_deg;
    math::SquareMatrix mountAntToBodyRotation = math::SquareMatrix::identity(3);
    double mountOrthogonalTolerance = 1e-6;
};

struct SearchGridConfig
{
    std::size_t azimuthCount = 21;
    std::size_t elevationCount = 9;
    math::Angle azimuthStart = -42_deg;
    math::Angle azimuthStep = 4.2_deg;
    math::Angle elevationStart = 3_deg;
    math::Angle elevationStep = 4.4_deg;
    math::Angle beamWidth = 6_deg;
};

// 확인/추적 빔 요청이 자신의 시각 이후 이 시간 이상 지나면 만료로 보고 폐기한다.
struct RequestQueueConfig
{
    std::chrono::milliseconds confirmationMaxDelay{20};
    std::chrono::milliseconds trackingMaxDelay{30};
};

struct RequestBeamConfig
{
    math::Angle confirmationBeamWidth = 3_deg;
    math::Angle trackingBeamWidth = 3_deg;
};

struct BeamConfig
{
    AttitudeConfig attitude;
    SearchGridConfig searchGrid;
    RequestQueueConfig requestQueue;
    RequestBeamConfig requestBeam;
};

} // namespace beam
