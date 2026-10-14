#include "beam_table.hpp"
#include <beam/config/beam_config.hpp>

#include <cstdint>
#include <stdexcept>
#include <limits>

namespace beam
{
namespace
{
using namespace math::literals;

// 축의 마지막 점(start + step * (count - 1))이 유한한지 확인한다.
// start와 step이 각각 유한해도 곱셈이나 덧셈에서 오버플로할 수 있다.
// 마지막 점이 유한하면 그 사이의 점도 모두 유한하다.
// 계산식은 BeamTable 생성자의 각도 계산과 같아야 한다.
bool isLastPointFinite(math::Angle start, math::Angle step, std::size_t count)
{
    return (start + step * static_cast<double>(count - 1)).isFinite();
}

void validate(const SearchGridConfig& config)
{
    if(config.azimuthCount == 0 || config.elevationCount == 0)
    {
        throw std::invalid_argument("SearchGridConfig: count must be greater than 0");
    }

    if(config.azimuthCount > std::numeric_limits<std::uint32_t>::max() / config.elevationCount)
    {
        throw std::invalid_argument("SearchGridConfig: beam count exceeds uint32 range");
    }

    if(!config.azimuthStart.isFinite() || !config.azimuthStep.isFinite() ||
       !config.elevationStart.isFinite() || !config.elevationStep.isFinite() ||
       !config.beamWidth.isFinite())
    {
        throw std::invalid_argument("SearchGridConfig: angle must be finite");
    }

    if(!isLastPointFinite(config.azimuthStart, config.azimuthStep, config.azimuthCount) ||
       !isLastPointFinite(config.elevationStart, config.elevationStep, config.elevationCount))
    {
        throw std::invalid_argument("SearchGridConfig: last grid angle must be finite");
    }

    if(config.beamWidth <= 0_deg)
    {
        throw std::invalid_argument("SearchGridConfig: beamWidth must be greater than 0");
    }
}
} // namespace

BeamTable::BeamTable(const AntEnuTransform& transform, const SearchGridConfig& config)
{
    validate(config);
    _beams.reserve(config.azimuthCount * config.elevationCount);

    for (std::size_t e = 0; e < config.elevationCount; ++e)
    {
        for (std::size_t a = 0; a < config.azimuthCount; ++a)
        {
            const math::SphericalDirection direction_ant = transform.enuToAnt(math::SphericalDirection(
                (config.azimuthStart + config.azimuthStep * static_cast<double>(a)).wrap180(),
                config.elevationStart + config.elevationStep * static_cast<double>(e)));

            BeamInfo beam;
            beam.beamId = static_cast<uint32_t>(_beams.size() + 1);
            beam.azimuth_ant = direction_ant.azimuth;
            beam.elevation_ant = direction_ant.elevation;
            beam.azimuthBeamWidth = config.beamWidth;
            beam.elevationBeamWidth = config.beamWidth;
            _beams.push_back(beam);
        }
    }
}

const BeamInfo& BeamTable::get(std::size_t index) const
{
    return _beams.at(index);
}

std::size_t BeamTable::size(void) const
{
    return _beams.size();
}

} // namespace beam
