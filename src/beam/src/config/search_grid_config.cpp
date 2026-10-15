#include <beam/config/search_grid_config.hpp>

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace beam
{
namespace
{
using namespace math::literals;

// 축의 마지막 점(start + step * (count - 1))을 계산한다.
// start와 step이 각각 유한해도 곱셈이나 덧셈에서 오버플로할 수 있고, 이때 Angle이 std::invalid_argument를 던진다.
// 마지막 점이 유한하면 그 사이의 점도 모두 유한하다.
// 계산식은 SearchGridConfig를 사용해 격자를 만드는 쪽의 각도 계산과 같아야 한다.
math::Angle lastPoint(math::Angle start, math::Angle step, std::size_t count)
{
    return start + step * static_cast<double>(count - 1);
}

bool isWithinElevationRange(math::Angle elevation)
{
    return elevation >= -90_deg && elevation <= 90_deg;
}

void validate(
    std::size_t azimuthCount,
    std::size_t elevationCount,
    math::Angle azimuthStart,
    math::Angle azimuthStep,
    math::Angle elevationStart,
    math::Angle elevationStep,
    math::Angle beamWidth
) {
    if (azimuthCount == 0 || elevationCount == 0)
    {
        throw std::invalid_argument("SearchGridConfig: count must be greater than 0");
    }

    // 개수가 0이 아님을 확인한 뒤에 나눗셈을 해야 한다.
    if (azimuthCount > std::numeric_limits<std::uint32_t>::max() / elevationCount)
    {
        throw std::invalid_argument("SearchGridConfig: beam count exceeds uint32 range");
    }

    // 마지막 점 계산이 오버플로하면 Angle이 std::invalid_argument를 던진다.
    (void)lastPoint(azimuthStart, azimuthStep, azimuthCount);
    const math::Angle elevationLast = lastPoint(elevationStart, elevationStep, elevationCount);

    // 고각 축은 단조 증가 또는 감소이므로 양 끝이 범위 안이면 중간 점도 범위 안이다.
    // 방위각은 SphericalDirection에 넘기기 전에 wrap180()으로 정규화하므로 범위를 검사하지 않는다.
    if (!isWithinElevationRange(elevationStart) || !isWithinElevationRange(elevationLast))
    {
        throw std::invalid_argument("SearchGridConfig: elevation must be in [-90, 90] degrees");
    }

    if (beamWidth <= 0_deg)
    {
        throw std::invalid_argument("SearchGridConfig: beamWidth must be greater than 0");
    }
}
} // namespace

SearchGridConfig::SearchGridConfig() : SearchGridConfig(Builder().build())
{
}

SearchGridConfig::SearchGridConfig(
    std::size_t azimuthCount,
    std::size_t elevationCount,
    math::Angle azimuthStart,
    math::Angle azimuthStep,
    math::Angle elevationStart,
    math::Angle elevationStep,
    math::Angle beamWidth
) : _azimuthCount(azimuthCount),
    _elevationCount(elevationCount),
    _azimuthStart(azimuthStart),
    _azimuthStep(azimuthStep),
    _elevationStart(elevationStart),
    _elevationStep(elevationStep),
    _beamWidth(beamWidth)
{
    validate(
        _azimuthCount,
        _elevationCount,
        _azimuthStart,
        _azimuthStep,
        _elevationStart,
        _elevationStep,
        _beamWidth);
}

std::size_t SearchGridConfig::azimuthCount() const noexcept
{
    return _azimuthCount;
}

std::size_t SearchGridConfig::elevationCount() const noexcept
{
    return _elevationCount;
}

math::Angle SearchGridConfig::azimuthStart() const noexcept
{
    return _azimuthStart;
}

math::Angle SearchGridConfig::azimuthStep() const noexcept
{
    return _azimuthStep;
}

math::Angle SearchGridConfig::elevationStart() const noexcept
{
    return _elevationStart;
}

math::Angle SearchGridConfig::elevationStep() const noexcept
{
    return _elevationStep;
}

math::Angle SearchGridConfig::beamWidth() const noexcept
{
    return _beamWidth;
}

SearchGridConfig SearchGridConfig::Builder::build() const
{
    return SearchGridConfig(
        _azimuthCount,
        _elevationCount,
        _azimuthStart,
        _azimuthStep,
        _elevationStart,
        _elevationStep,
        _beamWidth);
}

} // namespace beam
