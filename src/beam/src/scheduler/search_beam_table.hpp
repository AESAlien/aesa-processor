#pragma once
#include "attitude/ant_enu_transform.hpp"
#include <beam/config/search_grid_config.hpp>
#include "search_beam.hpp"
#include <vector>
#include <cstddef>

namespace beam
{

class SearchBeamTable
{
public:
    explicit SearchBeamTable(const AntEnuTransform& transform, const SearchGridConfig&);
    const SearchBeam& get(std::size_t index) const;
    std::size_t size(void) const;

    // 탐색 순서상 다음 빔을 반환하고 커서를 한 칸 옮긴다. 마지막 빔 다음에는 처음 빔으로 돌아간다.
    // get()과 size()는 커서와 무관하게 동작한다.
    const SearchBeam& nextBeam();

private:
    std::vector<SearchBeam> _beams{};
    std::size_t _searchCursor = 0;
};

} // namespace beam
