#pragma once
#include "attitude/ant_enu_transform.hpp"
#include <beam/config/beam_config.hpp>
#include "beam_info.hpp"
#include <vector>
#include <cstddef>

namespace beam
{

class BeamTable
{
public:
    explicit BeamTable(const AntEnuTransform& transform, const SearchGridConfig&);
    const BeamInfo& get(std::size_t index) const;
    std::size_t size(void) const;

private:
    std::vector<BeamInfo> _beams{};
};

} // namespace beam
