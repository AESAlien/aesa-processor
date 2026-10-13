#pragma once

#include <math/quantity/angle.hpp>
#include <math/linear_algebra/square_matrix.hpp>

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

struct RequestBeamConfig
{
    math::Angle confirmationBeamWidth = 3_deg;
    math::Angle trackingBeamWidth = 3_deg;
};

struct BeamConfig
{
    AttitudeConfig attitude;
    SearchGridConfig searchGrid;
    RequestBeamConfig requestBeam;
};

} // namespace beam
