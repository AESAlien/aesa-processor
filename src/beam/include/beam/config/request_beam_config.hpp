#pragma once

#include <math/quantity/angle.hpp>

#include <cstddef>

namespace aesa_processor::beam
{

using aesa_processor::math::literals::operator""_deg;

class RequestBeamConfig
{
public:
    class Builder
    {
    public:
        Builder& confirmationBeamWidth(aesa_processor::math::Angle value)   { _confirmationBeamWidth = value; return *this; }
        Builder& trackingBeamWidth(aesa_processor::math::Angle value)       { _trackingBeamWidth = value; return *this; }

        RequestBeamConfig build() const;
    private:
        aesa_processor::math::Angle _confirmationBeamWidth = 3_deg;
        aesa_processor::math::Angle _trackingBeamWidth = 3_deg;
    }; // class Builder

    RequestBeamConfig();
    RequestBeamConfig(aesa_processor::math::Angle confirmationBeamWidth,
                      aesa_processor::math::Angle trackingBeamWidth);
    
    aesa_processor::math::Angle confirmationBeamWidth() const noexcept;
    aesa_processor::math::Angle trackingBeamWidth() const noexcept;
private:
    void validate() const;

    aesa_processor::math::Angle _confirmationBeamWidth;
    aesa_processor::math::Angle _trackingBeamWidth;
};

} // namespace aesa_processor::beam