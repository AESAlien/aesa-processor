#pragma once

#include <beam/config/attitude_config.hpp>
#include <beam/config/search_grid_config.hpp>
#include <beam/config/request_beam_config.hpp>

#include <cstddef>
#include <optional>
#include <utility>

namespace beam
{
using math::literals::operator""_deg;

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

    BeamConfig();
    BeamConfig(AttitudeConfig attitude,
               SearchGridConfig searchGrid,
               RequestBeamConfig requestBeam);

    const AttitudeConfig attitude;
    const SearchGridConfig searchGrid;
    const RequestBeamConfig requestBeam;
};

} // namespace beam
