#include "search_grid.hpp"

#include <cstdint>

namespace beam
{

SearchGrid::SearchGrid(const AntEnuTransform& transform, const SearchGridConfig& config)
{
    _beams.reserve(config.azimuthCount() * config.elevationCount());

    for (std::size_t e = 0; e < config.elevationCount(); ++e)
    {
        for (std::size_t a = 0; a < config.azimuthCount(); ++a)
        {
            // 방위각은 SphericalDirection이 [0, 360)으로 정규화한다.
            const math::SphericalDirection direction_ant = transform.antToEnu(math::SphericalDirection(
                config.azimuthStart() + config.azimuthStep() * static_cast<double>(a),
                config.elevationStart() + config.elevationStep() * static_cast<double>(e)));

            _beams.push_back(SearchBeam{
                static_cast<uint32_t>(_beams.size() + 1),
                direction_ant,
                config.beamWidth(),
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

} // namespace beam
