#pragma once

#include <beam/config/attitude_config.hpp>
#include <beam/config/search_grid_config.hpp>
#include <beam/config/request_beam_config.hpp>

#include <cstddef>

namespace aesa_processor::beam
{
using aesa_processor::math::literals::operator""_deg;

class BeamConfig
{
public:
    class Builder
    {
    public:
        Builder& attitudeConfig(AttitudeConfig value)       { _attitude = value; return *this; }
        Builder& searchGridConfig(SearchGridConfig value)   { _searchGrid = value; return *this; }
        Builder& requestBeamConfig(RequestBeamConfig value) { _requestBeam = value; return *this; }

        BeamConfig build() const;
    private:
        AttitudeConfig _attitude;
        SearchGridConfig _searchGrid;
        RequestBeamConfig _requestBeam;
    }; // class Builder

    BeamConfig();
    BeamConfig(AttitudeConfig attitude,
               SearchGridConfig searchGrid,
               RequestBeamConfig requestBeam);

    const AttitudeConfig& attitude() const noexcept;
    const SearchGridConfig& searchGrid() const noexcept;
    const RequestBeamConfig& requestBeam() const noexcept;
private:
    AttitudeConfig _attitude;
    SearchGridConfig _searchGrid;
    RequestBeamConfig _requestBeam;
};

} // namespace aesa_processor::beam
