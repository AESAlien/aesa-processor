#pragma once

#include <math/bounded/speed.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <optional>

namespace target
{

class AssociationConfig
{
public:
    class Builder
    {
    public:
        Builder& maxAssociationSpeed(math::LinearVelocity value)
        {
            _maxAssociationSpeed = value;
            return *this;
        }

        Builder& maxInitialDopplerDifference(math::LinearVelocity value)
        {
            _maxInitialDopplerDifference = value;
            return *this;
        }

        Builder& positionNdsThreshold(double value)
        {
            _positionNdsThreshold = value;
            return *this;
        }

        AssociationConfig build() const;

    private:
        std::optional<math::LinearVelocity> _maxAssociationSpeed;
        std::optional<math::LinearVelocity> _maxInitialDopplerDifference;
        std::optional<double> _positionNdsThreshold;
    };

    AssociationConfig(
        math::Speed maxAssociationSpeed,
        math::Speed maxInitialDopplerDifference,
        double positionNdsThreshold
    );

    AssociationConfig(
        math::LinearVelocity maxAssociationSpeed,
        math::LinearVelocity maxInitialDopplerDifference,
        double positionNdsThreshold
    );

    const math::Speed& maxAssociationSpeed() const noexcept;
    const math::Speed& maxInitialDopplerDifference() const noexcept;
    double positionNdsThreshold() const noexcept;

private:
    math::Speed _maxAssociationSpeed;
    math::Speed _maxInitialDopplerDifference;
    double _positionNdsThreshold;
    void validate() const;
};

} // namespace target
