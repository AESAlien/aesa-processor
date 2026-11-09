#pragma once

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
        math::LinearVelocity maxAssociationSpeed,
        math::LinearVelocity maxInitialDopplerDifference,
        double positionNdsThreshold
    );

    const math::LinearVelocity& maxAssociationSpeed() const noexcept;
    const math::LinearVelocity& maxInitialDopplerDifference() const noexcept;
    double positionNdsThreshold() const noexcept;

private:
    math::LinearVelocity _maxAssociationSpeed;
    math::LinearVelocity _maxInitialDopplerDifference;
    double _positionNdsThreshold;
    void validate() const;
};

} // namespace target
