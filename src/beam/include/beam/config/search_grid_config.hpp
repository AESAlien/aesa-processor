#pragma once

#include <math/quantity/angle.hpp>

#include <cstddef>

namespace aesa_processor::beam
{
using aesa_processor::math::literals::operator""_deg;

class SearchGridConfig
{
public:
    class Builder
    {
    public:
        Builder& azimuthCount(std::size_t value)    { _azimuthCount = value; return *this; }
        Builder& elevationCount(std::size_t value)  { _elevationCount = value; return *this; }
        Builder& azimuthStart(aesa_processor::math::Angle value)    { _azimuthStart = value; return *this; }
        Builder& azimuthStep(aesa_processor::math::Angle value)     { _azimuthStep = value; return *this; }
        Builder& elevationStart(aesa_processor::math::Angle value)  { _elevationStart = value; return *this; }
        Builder& elevationStep(aesa_processor::math::Angle value)   { _elevationStep = value; return *this; }
        Builder& beamWidth(aesa_processor::math::Angle value)       { _beamWidth = value; return *this; }

        SearchGridConfig build() const;

    private:
        std::size_t _azimuthCount = 21;
        std::size_t _elevationCount = 9;
        aesa_processor::math::Angle _azimuthStart = -42_deg;
        aesa_processor::math::Angle _azimuthStep = 4.2_deg;
        aesa_processor::math::Angle _elevationStart = 3_deg;
        aesa_processor::math::Angle _elevationStep = 4.4_deg;
        aesa_processor::math::Angle _beamWidth = 6_deg;
    }; // class Builder

    SearchGridConfig();
    SearchGridConfig(std::size_t azimuthCount, std::size_t elevationCount,
                     aesa_processor::math::Angle azimuthStart, aesa_processor::math::Angle azimuthStep,
                     aesa_processor::math::Angle elevationStart, aesa_processor::math::Angle elevationStep,
                     aesa_processor::math::Angle beamWidth);

    std::size_t azimuthCount() const noexcept;
    std::size_t elevationCount() const noexcept;
    aesa_processor::math::Angle azimuthStart() const noexcept;
    aesa_processor::math::Angle azimuthStep() const noexcept;
    aesa_processor::math::Angle elevationStart() const noexcept;
    aesa_processor::math::Angle elevationStep() const noexcept;
    aesa_processor::math::Angle beamWidth() const noexcept;
private:
    void validate() const;

    std::size_t _azimuthCount;
    std::size_t _elevationCount;
    aesa_processor::math::Angle _azimuthStart;
    aesa_processor::math::Angle _azimuthStep;
    aesa_processor::math::Angle _elevationStart;
    aesa_processor::math::Angle _elevationStep;
    aesa_processor::math::Angle _beamWidth;
};

} // namespace aesa_processor::beam