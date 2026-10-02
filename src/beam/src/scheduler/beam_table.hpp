#pragma once
#include "beam_info.hpp"
#include <array>
#include <beam/domain/antenna_to_enu.hpp>
#include <cstddef>

namespace beam
{

class BeamTable
{
public:
    explicit BeamTable(const AntennaToEnu& in);
    const BeamInfo& Get(std::size_t idx) const;
    static constexpr std::size_t Size()
    {
        return 189;
    }

private:
    static constexpr std::size_t Count = 189;
    std::array<BeamInfo, Count> _beams{};
};

} // namespace beam
