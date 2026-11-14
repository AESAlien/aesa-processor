#include "search/search_grid.hpp"

#include <pose/domain/ant_enu_transform.hpp>

#include <cstdint>

namespace beam
{

SearchGrid::SearchGrid(const pose::AntEnuTransform& transform, const SearchGridConfig& config)
{
    _beams.reserve(config.azimuthCount() * config.elevationCount());

    for (std::size_t e = 0; e < config.elevationCount(); ++e)
    {
        for (std::size_t a = 0; a < config.azimuthCount(); ++a)
        {
            const math::SphericalDirection direction_enu = transform.antToEnu(math::SphericalDirection(
                config.azimuthAt(a),
                config.elevationAt(e)));

            _beams.push_back(SearchBeam{
                static_cast<uint32_t>(_beams.size()),
                direction_enu,
                config.beamWidth()});
        }
    }
}

const SearchBeam& SearchGrid::nextBeam()
{
    const SearchBeam& beam = _beams[_searchCursor];
    _searchCursor = (_searchCursor + 1) % _beams.size();
    return beam;
}

void SearchGrid::reset() noexcept
{
    _searchCursor = 0;
}

} // namespace beam
