#include "beam_table.hpp"

namespace beam
{
namespace
{
constexpr std::size_t AZIMUTH_COUNT = 21;
constexpr std::size_t ELEVATION_COUNT = 9;
constexpr double AZIMUTH_START_DEG = -42.0, AZIMUTH_STEP_DEG = 4.2;
constexpr double ELEVATION_START_DEG = 3.0, ELEVATION_STEP_DEG = 4.4;
constexpr float BEAM_WIDTH_DEG = 6.0f;
} // namespace

BeamTable::BeamTable(const AntEnuTransform& transform)
{
    static_assert(AZIMUTH_COUNT * ELEVATION_COUNT == 189, "grid size must match BeamTable::size()");

    for (std::size_t e = 0; e < ELEVATION_COUNT; ++e)
    {
        for (std::size_t a = 0; a < AZIMUTH_COUNT; ++a)
        {
            const std::size_t index = e * AZIMUTH_COUNT + a;

            const auto [azimuth_ant_deg, elevation_ant_deg] = transform.enuToAnt(
                AZIMUTH_START_DEG + AZIMUTH_STEP_DEG * a, ELEVATION_START_DEG + ELEVATION_STEP_DEG * e);

            BeamInfo& beam = _beams[index];
            beam.beamId = static_cast<std::uint32_t>(index + 1);
            beam.azimuth_ant_deg = static_cast<float>(azimuth_ant_deg);
            beam.elevation_ant_deg = static_cast<float>(elevation_ant_deg);
            beam.azimuthBeamWidth_deg = BEAM_WIDTH_DEG;
            beam.elevationBeamWidth_deg = BEAM_WIDTH_DEG;
        }
    }
}

const BeamInfo& BeamTable::get(std::size_t index) const
{
    return _beams.at(index);
}

} // namespace beam
