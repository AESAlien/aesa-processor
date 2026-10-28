#pragma once

#include <beam/domain/beam_width.hpp>

#include <math/quantity/angle.hpp>
#include <math/geometry/azimuth.hpp>
#include <math/geometry/elevation.hpp>

#include <cstddef>
#include <optional>

namespace beam
{
struct SearchGridConfig
{
public:
    class Builder
    {
    public:
        Builder& azimuthCount(std::size_t value)    { _azimuthCount = value; return *this; }
        Builder& elevationCount(std::size_t value)  { _elevationCount = value; return *this; }
        Builder& azimuthStart(math::Azimuth value)  { _azimuthStart = value; return *this; }
        Builder& azimuthStep(math::Angle value)     { _azimuthStep = value; return *this; }
        Builder& elevationStart(math::Elevation value) { _elevationStart = value; return *this; }
        Builder& elevationStep(math::Angle value)   { _elevationStep = value; return *this; }
        Builder& beamWidth(BeamWidth value)         { _beamWidth = value; return *this; }

        SearchGridConfig build() const;

    private:
        std::optional<std::size_t> _azimuthCount;
        std::optional<std::size_t> _elevationCount;
        std::optional<math::Azimuth> _azimuthStart;
        std::optional<math::Angle> _azimuthStep;
        std::optional<math::Elevation> _elevationStart;
        std::optional<math::Angle> _elevationStep;
        std::optional<BeamWidth> _beamWidth;
    }; // class Builder

    SearchGridConfig(std::size_t azimuthCount, std::size_t elevationCount,
                     math::Azimuth azimuthStart, math::Angle azimuthStep,
                     math::Elevation elevationStart, math::Angle elevationStep,
                     BeamWidth beamWidth);

    const std::size_t azimuthCount;
    const std::size_t elevationCount;
    const math::Azimuth azimuthStart;
    const math::Angle azimuthStep;
    const math::Elevation elevationStart;
    const math::Angle elevationStep;
    const BeamWidth beamWidth;

private:
    void validate() const;
};

} // namespace beam
