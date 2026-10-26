#pragma once

#include <math/quantity/angle.hpp>

#include <cstddef>

namespace beam
{

using math::literals::operator""_deg;

struct RequestBeamConfig
{
public:
    class Builder
    {
    public:
        Builder& confirmationBeamWidth(math::Angle value)   { _confirmationBeamWidth = value; return *this; }
        Builder& trackingBeamWidth(math::Angle value)       { _trackingBeamWidth = value; return *this; }

        RequestBeamConfig build() const;
    private:
        math::Angle _confirmationBeamWidth = 3_deg;
        math::Angle _trackingBeamWidth = 3_deg;
    }; // class Builder

    RequestBeamConfig();
    RequestBeamConfig(math::Angle confirmationBeamWidth,
                      math::Angle trackingBeamWidth);
    
    const math::Angle confirmationBeamWidth;
    const math::Angle trackingBeamWidth;

private:
    void validate() const;
};

} // namespace beam