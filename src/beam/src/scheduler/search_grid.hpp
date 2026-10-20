#pragma once
#include "attitude/ant_enu_transform.hpp"
#include <beam/config/search_grid_config.hpp>
#include "search_beam.hpp"
#include <vector>
#include <cstddef>

namespace beam
{

// SearchGridConfig의 격자(안테나 정면 기준)를 ENU 기준 탐색 빔 목록으로 만들어 두고, 탐색 순서대로 하나씩 꺼내 쓴다.
class SearchGrid
{
public:
    explicit SearchGrid(const AntEnuTransform& transform, const SearchGridConfig& config);

    // 탐색 순서상 다음 빔을 반환하고 커서를 한 칸 옮긴다. 마지막 빔 다음에는 처음 빔으로 돌아간다.
    const SearchBeam& nextBeam();

private:
    std::vector<SearchBeam> _beams{};
    std::size_t _searchCursor = 0;
};

} // namespace beam
