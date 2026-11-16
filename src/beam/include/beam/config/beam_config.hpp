#pragma once

#include <beam/config/search_grid_config.hpp>
#include <beam/config/request_beam_config.hpp>
#include <beam/config/request_coverage_config.hpp>

#include <optional>
#include <utility>

namespace beam
{
struct BeamConfig
{
public:
    class Builder
    {
    public:
        Builder& searchGridConfig(SearchGridConfig value)   { _searchGrid.emplace(std::move(value)); return *this; }
        Builder& requestBeamConfig(RequestBeamConfig value) { _requestBeam.emplace(std::move(value)); return *this; }
        Builder& requestCoverageConfig(RequestCoverageConfig value) { _requestCoverage.emplace(std::move(value)); return *this; }

        BeamConfig build() const;
    private:
        std::optional<SearchGridConfig> _searchGrid;
        std::optional<RequestBeamConfig> _requestBeam;
        std::optional<RequestCoverageConfig> _requestCoverage;
    }; // class Builder

    BeamConfig(
        SearchGridConfig searchGrid,
        RequestBeamConfig requestBeam,
        RequestCoverageConfig requestCoverage
    );

    SearchGridConfig searchGrid;
    RequestBeamConfig requestBeam;
    RequestCoverageConfig requestCoverage;
};

} // namespace beam
