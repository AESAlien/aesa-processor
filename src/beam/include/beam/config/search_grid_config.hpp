#pragma once

#include <math/quantity/angle.hpp>

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
        Builder& azimuthStart(math::Angle value)    { _azimuthStart = value; return *this; }
        Builder& azimuthStep(math::Angle value)     { _azimuthStep = value; return *this; }
        Builder& elevationStart(math::Angle value)  { _elevationStart = value; return *this; }
        Builder& elevationStep(math::Angle value)   { _elevationStep = value; return *this; }
        Builder& beamWidth(math::Angle value)       { _beamWidth = value; return *this; }

        SearchGridConfig build() const;

    private:
        std::optional<std::size_t> _azimuthCount;
        std::optional<std::size_t> _elevationCount;
        std::optional<math::Angle> _azimuthStart;
        std::optional<math::Angle> _azimuthStep;
        std::optional<math::Angle> _elevationStart;
        std::optional<math::Angle> _elevationStep;
        std::optional<math::Angle> _beamWidth;
    }; // class Builder

    SearchGridConfig(std::size_t azimuthCount, std::size_t elevationCount,
                     math::Angle azimuthStart, math::Angle azimuthStep,
                     math::Angle elevationStart, math::Angle elevationStep,
                     math::Angle beamWidth);

    const std::size_t azimuthCount;
    const std::size_t elevationCount;
    const math::Angle azimuthStart;
    const math::Angle azimuthStep;
    const math::Angle elevationStart;
    const math::Angle elevationStep;
    const math::Angle beamWidth;

private:
    void validate() const;
};

} // namespace beam
