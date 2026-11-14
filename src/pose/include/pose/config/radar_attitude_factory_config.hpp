#pragma once

#include <math/quantity/angle.hpp>

#include <optional>

namespace pose
{

class RadarAttitudeFactoryConfig
{
public:
    class Builder
    {
    public:
        Builder& maxAbsPitch(math::Angle value);

        RadarAttitudeFactoryConfig build() const;

    private:
        std::optional<math::Angle> _maxAbsPitch;
    };

    explicit RadarAttitudeFactoryConfig(math::Angle maxAbsPitch);

    const math::Angle& maxAbsPitch() const noexcept;

private:
    math::Angle _maxAbsPitch;
    void validate() const;
};

} // namespace pose
