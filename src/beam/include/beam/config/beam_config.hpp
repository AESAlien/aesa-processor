#pragma once

#include <beam/config/attitude_config.hpp>
#include <beam/config/search_grid_config.hpp>
#include <beam/config/request_beam_config.hpp>

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
        Builder& attitudeConfig(AttitudeConfig value)       { _attitude.emplace(std::move(value)); return *this; }
        Builder& searchGridConfig(SearchGridConfig value)   { _searchGrid.emplace(std::move(value)); return *this; }
        Builder& requestBeamConfig(RequestBeamConfig value) { _requestBeam.emplace(std::move(value)); return *this; }

        BeamConfig build() const;
    private:
        std::optional<AttitudeConfig> _attitude;
        std::optional<SearchGridConfig> _searchGrid;
        std::optional<RequestBeamConfig> _requestBeam;
    }; // class Builder

    BeamConfig(AttitudeConfig attitude,
               SearchGridConfig searchGrid,
               RequestBeamConfig requestBeam);

    AttitudeConfig attitude;
    SearchGridConfig searchGrid;
    RequestBeamConfig requestBeam;
};

} // namespace beam
