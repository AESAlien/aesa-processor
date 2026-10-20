#pragma once

#include <math/quantity/angle.hpp>

#include <cstddef>

namespace beam
{

using math::literals::operator""_deg;

class RequestBeamConfig
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
    
    math::Angle confirmationBeamWidth() const noexcept;
    math::Angle trackingBeamWidth() const noexcept;
private:
    void validate() const;

    math::Angle _confirmationBeamWidth;
    math::Angle _trackingBeamWidth;
};

} // namespace beam