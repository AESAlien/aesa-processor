#pragma once

#include <math/quantity/angle.hpp>

#include <cstddef>

namespace beam
{
using math::literals::operator""_deg;

class SearchGridConfig
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
        std::size_t _azimuthCount = 21;
        std::size_t _elevationCount = 9;
        math::Angle _azimuthStart = -42_deg;
        math::Angle _azimuthStep = 4.2_deg;
        math::Angle _elevationStart = 3_deg;
        math::Angle _elevationStep = 4.4_deg;
        math::Angle _beamWidth = 6_deg;
    }; // class Builder

    SearchGridConfig();
    SearchGridConfig(std::size_t azimuthCount, std::size_t elevationCount,
                     math::Angle azimuthStart, math::Angle azimuthStep,
                     math::Angle elevationStart, math::Angle elevationStep,
                     math::Angle beamWidth);

    std::size_t azimuthCount() const noexcept;
    std::size_t elevationCount() const noexcept;
    math::Angle azimuthStart() const noexcept;
    math::Angle azimuthStep() const noexcept;
    math::Angle elevationStart() const noexcept;
    math::Angle elevationStep() const noexcept;
    math::Angle beamWidth() const noexcept;
private:
    void validate() const;

    std::size_t _azimuthCount;
    std::size_t _elevationCount;
    math::Angle _azimuthStart;
    math::Angle _azimuthStep;
    math::Angle _elevationStart;
    math::Angle _elevationStep;
    math::Angle _beamWidth;
};

} // namespace beam