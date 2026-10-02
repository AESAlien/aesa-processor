#pragma once
#include "beam_info.hpp"
#include <beam/dto/antenna_to_enu.hpp>
#include <array>
#include <cstddef>

namespace beam
{

class BeamTable
{
public:
    explicit BeamTable(const AntennaToEnu& in);
    const BeamInfo& get(std::size_t idx) const;
    static constexpr std::size_t size() { return 189; }
private:
    static constexpr std::size_t kCount = 189;
    std::array<BeamInfo, kCount> beams_{};
};

}   // namespace beam
