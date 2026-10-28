#pragma once

#include <beam/domain/beam_width.hpp>

#include <cstddef>
#include <optional>

namespace beam
{

struct RequestBeamConfig
{
public:
    class Builder
    {
    public:
        Builder& confirmationBeamWidth(BeamWidth value) { _confirmationBeamWidth = value; return *this; }
        Builder& trackingBeamWidth(BeamWidth value) { _trackingBeamWidth = value; return *this; }

        RequestBeamConfig build() const;
    private:
        std::optional<BeamWidth> _confirmationBeamWidth;
        std::optional<BeamWidth> _trackingBeamWidth;
    }; // class Builder

    RequestBeamConfig(
        BeamWidth confirmationBeamWidth,
        BeamWidth trackingBeamWidth);
    
    BeamWidth confirmationBeamWidth;
    BeamWidth trackingBeamWidth;
};

} // namespace beam
