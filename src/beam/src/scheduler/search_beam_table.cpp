#include "search_beam_table.hpp"

#include <cstdint>

namespace beam
{

SearchBeamTable::SearchBeamTable(const AntEnuTransform& transform, const SearchGridConfig& config)
{
    _beams.reserve(config.azimuthCount() * config.elevationCount());

    for (std::size_t e = 0; e < config.elevationCount(); ++e)
    {
        for (std::size_t a = 0; a < config.azimuthCount(); ++a)
        {
            const math::SphericalDirection direction_ant = transform.enuToAnt(math::SphericalDirection(
                math::wrap180(config.azimuthStart() + config.azimuthStep() * static_cast<double>(a)),
                config.elevationStart() + config.elevationStep() * static_cast<double>(e)));

            _beams.push_back(SearchBeam{
                static_cast<uint32_t>(_beams.size() + 1),
                direction_ant,
                config.beamWidth(),
                config.beamWidth()});
        }
    }
}

const SearchBeam& SearchBeamTable::get(std::size_t index) const
{
    return _beams.at(index);
}

std::size_t SearchBeamTable::size(void) const
{
    return _beams.size();
}

const SearchBeam& SearchBeamTable::nextBeam()
{
    const SearchBeam& beam = _beams[_searchCursor];
    _searchCursor = (_searchCursor + 1) % _beams.size();
    return beam;
}

} // namespace beam
