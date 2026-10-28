#pragma once

#include <beam/domain/beam_width.hpp>

#include <math/quantity/angle.hpp>
#include <math/geometry/azimuth.hpp>
#include <math/geometry/elevation.hpp>

#include <cstddef>
#include <optional>

namespace beam
{
class SearchGridConfig
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

    std::size_t azimuthCount() const noexcept;
    std::size_t elevationCount() const noexcept;
    const math::Azimuth& azimuthStart() const noexcept;
    const math::Angle& azimuthStep() const noexcept;
    const math::Elevation& elevationStart() const noexcept;
    const math::Angle& elevationStep() const noexcept;
    const BeamWidth& beamWidth() const noexcept;

    // 격자의 index번째(0부터) 방위각/고각. start + step * index로 계산한다.
    // 격자 점의 각도 계산은 이 두 함수에만 둔다. index가 count 이상인지는 검사하지 않는다.
    // 방위각은 Azimuth가 [0, 360)으로 정규화하고, 고각이 [-90, 90]도를 벗어나면 Elevation이 std::invalid_argument를 던진다.
    math::Azimuth azimuthAt(std::size_t index) const;
    math::Elevation elevationAt(std::size_t index) const;

private:
    std::size_t _azimuthCount;
    std::size_t _elevationCount;
    math::Azimuth _azimuthStart;
    math::Angle _azimuthStep;
    math::Elevation _elevationStart;
    math::Angle _elevationStep;
    BeamWidth _beamWidth;
    void validate() const;
};

} // namespace beam
