#pragma once
#include "attitude/ant_enu_transform.hpp"
#include "beam_info.hpp"
#include <array>
#include <cstddef>

namespace beam
{

class BeamTable
{
public:
    explicit BeamTable(const AntEnuTransform& transform);
    const BeamInfo& get(std::size_t index) const;
    static constexpr std::size_t size()
    {
        return 189;
    }

private:
    static constexpr std::size_t COUNT = 189;
    std::array<BeamInfo, COUNT> _beams{};
};

} // namespace beam
