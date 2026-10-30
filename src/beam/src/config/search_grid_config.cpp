#include <beam/config/search_grid_config.hpp>

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace beam
{
using namespace math::literals;

SearchGridConfig::SearchGridConfig(
    std::size_t azimuthCount,
    std::size_t elevationCount,
    math::CircularAngle azimuthStart,
    math::Angle azimuthStep,
    math::TiltAngle elevationStart,
    math::Angle elevationStep,
    BeamWidth beamWidth
) : _azimuthCount(azimuthCount),
    _elevationCount(elevationCount),
    _azimuthStart(azimuthStart),
    _azimuthStep(azimuthStep),
    _elevationStart(elevationStart),
    _elevationStep(elevationStep),
    _beamWidth(beamWidth)
{
    validate();
}

std::size_t SearchGridConfig::azimuthCount() const noexcept
{
    return _azimuthCount;
}

std::size_t SearchGridConfig::elevationCount() const noexcept
{
    return _elevationCount;
}

const math::CircularAngle& SearchGridConfig::azimuthStart() const noexcept
{
    return _azimuthStart;
}

const math::Angle& SearchGridConfig::azimuthStep() const noexcept
{
    return _azimuthStep;
}

const math::TiltAngle& SearchGridConfig::elevationStart() const noexcept
{
    return _elevationStart;
}

const math::Angle& SearchGridConfig::elevationStep() const noexcept
{
    return _elevationStep;
}

const BeamWidth& SearchGridConfig::beamWidth() const noexcept
{
    return _beamWidth;
}

void SearchGridConfig::validate() const
{
    if (_azimuthCount == 0 || _elevationCount == 0)
    {
        throw std::invalid_argument("SearchGridConfig: count must be greater than 0");
    }

    // 개수가 0이 아님을 확인한 뒤에 나눗셈을 해야 한다.
    if (_azimuthCount > std::numeric_limits<std::uint32_t>::max() / _elevationCount)
    {
        throw std::invalid_argument("SearchGridConfig: beam count exceeds uint32 range");
    }

    // 각 축의 마지막 점을 계산해 본다. 계산이 오버플로하면 Angle이, 고각이 범위를 벗어나면 TiltAngle이
    // std::invalid_argument를 던진다.
    // 고각 축은 단조 증가 또는 감소이므로 양 끝이 범위 안이면 중간 점도 범위 안이다.
    // 방위각은 CircularAngle이 [0, 360)으로 정규화하므로 범위를 검사하지 않는다.
    (void)azimuthAt(_azimuthCount - 1);
    (void)elevationAt(_elevationCount - 1);
}

math::CircularAngle SearchGridConfig::azimuthAt(std::size_t index) const
{
    return _azimuthStart + _azimuthStep * static_cast<double>(index);
}

math::TiltAngle SearchGridConfig::elevationAt(std::size_t index) const
{
    return _elevationStart + _elevationStep * static_cast<double>(index);
}

SearchGridConfig SearchGridConfig::Builder::build() const
{
    if (!_azimuthCount || !_elevationCount || !_azimuthStart || !_azimuthStep ||
        !_elevationStart || !_elevationStep || !_beamWidth)
    {
        throw std::logic_error("SearchGridConfig::Builder requires every value");
    }

    return SearchGridConfig(
        *_azimuthCount,
        *_elevationCount,
        *_azimuthStart,
        *_azimuthStep,
        *_elevationStart,
        *_elevationStep,
        *_beamWidth);
}

} // namespace beam
