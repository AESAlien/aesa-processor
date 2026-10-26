#pragma once

#include <math/quantity/angle.hpp>

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
        Builder& confirmationBeamWidth(math::Angle value)   { _confirmationBeamWidth = value; return *this; }
        Builder& trackingBeamWidth(math::Angle value)       { _trackingBeamWidth = value; return *this; }

        RequestBeamConfig build() const;
    private:
        std::optional<math::Angle> _confirmationBeamWidth;
        std::optional<math::Angle> _trackingBeamWidth;
    }; // class Builder

    RequestBeamConfig(math::Angle confirmationBeamWidth,
                      math::Angle trackingBeamWidth);
    
    const math::Angle confirmationBeamWidth;
    const math::Angle trackingBeamWidth;

private:
    void validate() const;
};

} // namespace beam
