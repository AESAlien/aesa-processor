#include "beam_table.hpp"

namespace beam
{
namespace
{
constexpr std::size_t AzCount = 21;
constexpr std::size_t ElCount = 9;
constexpr double AzStart_deg = -42.0, AzStep_deg = 4.2;
constexpr double ElStart_deg = 3.0, ElStep_deg = 4.4;
constexpr float BeamWidth_deg = 6.0f;
} // namespace

BeamTable::BeamTable(const AntennaToEnu& in)
{
    static_assert(AzCount * ElCount == 189, "grid size must match BeamTable::Size()");

    for (std::size_t e = 0; e < ElCount; ++e)
    {
        for (std::size_t a = 0; a < AzCount; ++a)
        {
            const std::size_t idx = e * AzCount + a;

            double azAnt = 0.0, elAnt = 0.0;
            in.EnuToAntAngle(AzStart_deg + AzStep_deg * a, ElStart_deg + ElStep_deg * e, azAnt,
                             elAnt);

            BeamInfo& b = _beams[idx];
            b.beamId = static_cast<std::uint32_t>(idx + 1);
            b.az_deg = static_cast<float>(azAnt);
            b.el_deg = static_cast<float>(elAnt);
            b.azWidth_deg = BeamWidth_deg;
            b.elWidth_deg = BeamWidth_deg;
        }
    }
}

const BeamInfo& BeamTable::Get(std::size_t idx) const
{
    return _beams.at(idx);
}

} // namespace beam
