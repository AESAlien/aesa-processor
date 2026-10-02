#include "beam_table.hpp"

namespace beam
{
namespace
{
constexpr std::size_t kAzCount = 21;
constexpr std::size_t kElCount = 9;
constexpr double kAzStart_deg = -42.0, kAzStep_deg = 4.2;
constexpr double kElStart_deg = 3.0, kElStep_deg = 4.4;
constexpr float kBeamWidth_deg = 6.0f;
}   // namespace

BeamTable::BeamTable(const AntennaToEnu& in)
{
    static_assert(kAzCount * kElCount == 189, "grid size must match BeamTable::size()");

    for(std::size_t e=0; e<kElCount; ++e) {
        for(std::size_t a=0; a<kAzCount; ++a) {
            const std::size_t idx = e * kAzCount + a;

            double az_ant = 0.0, el_ant = 0.0;
            in.enuToAntAngle(kAzStart_deg + kAzStep_deg * a,
                kElStart_deg + kElStep_deg * e, az_ant, el_ant);
            
            BeamInfo& b = beams_[idx];
            b.beamID        = static_cast<std::uint32_t>(idx + 1);
            b.az_deg        = static_cast<float>(az_ant);
            b.el_deg        = static_cast<float>(el_ant);
            b.az_width_deg  = kBeamWidth_deg;
            b.el_width_deg  = kBeamWidth_deg;
        }
    }
}

const BeamInfo& BeamTable::get(std::size_t idx) const
{
    return beams_.at(idx);
}

}   // namespace beam

