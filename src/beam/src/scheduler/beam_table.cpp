#include "beam_table.hpp"

namespace beam
{
namespace
{
using namespace math::literals;

constexpr std::size_t AZIMUTH_COUNT = 21;
constexpr std::size_t ELEVATION_COUNT = 9;
const math::Angle AZIMUTH_START = -42_deg, AZIMUTH_STEP = 4.2_deg;
const math::Angle ELEVATION_START = 3_deg, ELEVATION_STEP = 4.4_deg;
const math::Angle BEAM_WIDTH = 6_deg;
} // namespace

BeamTable::BeamTable(const AntEnuTransform& transform)
{
    static_assert(AZIMUTH_COUNT * ELEVATION_COUNT == 189, "grid size must match BeamTable::size()");

    for (std::size_t e = 0; e < ELEVATION_COUNT; ++e)
    {
        for (std::size_t a = 0; a < AZIMUTH_COUNT; ++a)
        {
            const std::size_t index = e * AZIMUTH_COUNT + a;

            const auto [azimuth_ant, elevation_ant] = transform.enuToAnt(
                AZIMUTH_START + AZIMUTH_STEP * static_cast<double>(a),
                ELEVATION_START + ELEVATION_STEP * static_cast<double>(e)
            );

            BeamInfo& beam = _beams[index];
            beam.beamId = static_cast<uint32_t>(index + 1);
            beam.azimuth_ant = azimuth_ant;
            beam.elevation_ant = elevation_ant;
            beam.azimuthBeamWidth = BEAM_WIDTH;
            beam.elevationBeamWidth = BEAM_WIDTH;
        }
    }
}

const BeamInfo& BeamTable::get(std::size_t index) const
{
    return _beams.at(index);
}

} // namespace beam
