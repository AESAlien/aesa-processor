#include "beam_table.hpp"
#include <beam/config/beam_config.hpp>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <limits>

namespace beam
{
namespace
{
using namespace math::literals;

bool isFinite(math::Angle angle)
{
    return std::isfinite(angle.deg());
}

void validate(const SearchGridConfig& config)
{
    if(config.azimuthCount == 0 || config.elevationCount == 0) {
        throw std::invalid_argument("SearchGridConfig: count must be greater than 0");
    }

    if(config.azimuthCount > std::numeric_limits<std::uint32_t>::max() / config.elevationCount) {
        throw std::invalid_argument("SearchGridConfig: beam count exceeds uint32 range");
    }

    if(!isFinite(config.azimuthStart) || !isFinite(config.azimuthStep) ||
       !isFinite(config.elevationStart) || !isFinite(config.elevationStep) ||
       !isFinite(config.beamWidth)) {
        throw std::invalid_argument("SearchGridConfig: angle must be finite");
    }

    if(config.beamWidth <= 0_deg) {
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
            const auto [azimuth_ant, elevation_ant] = transform.enuToAnt(
                config.azimuthStart + config.azimuthStep * static_cast<double>(a),
                config.elevationStart + config.elevationStep * static_cast<double>(e)
            );

            BeamInfo beam;
            beam.beamId = static_cast<uint32_t>(_beams.size() + 1);
            beam.azimuth_ant = azimuth_ant;
            beam.elevation_ant = elevation_ant;
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
