#include <target/config/association_config.hpp>

#include <cmath>
#include <stdexcept>

namespace target
{

AssociationConfig::AssociationConfig(
    math::Speed maxAssociationSpeed,
    math::Speed maxInitialDopplerDifference,
    double positionNdsThreshold
) : _maxAssociationSpeed(maxAssociationSpeed),
    _maxInitialDopplerDifference(maxInitialDopplerDifference),
    _positionNdsThreshold(positionNdsThreshold)
{
    validate();
}

AssociationConfig::AssociationConfig(
    math::LinearVelocity maxAssociationSpeed,
    math::LinearVelocity maxInitialDopplerDifference,
    double positionNdsThreshold
) : AssociationConfig(
        math::Speed(maxAssociationSpeed),
        math::Speed(maxInitialDopplerDifference),
        positionNdsThreshold
    )
{
}

const math::Speed& AssociationConfig::maxAssociationSpeed() const noexcept
{
    return _maxAssociationSpeed;
}

const math::Speed& AssociationConfig::maxInitialDopplerDifference() const noexcept
{
    return _maxInitialDopplerDifference;
}

double AssociationConfig::positionNdsThreshold() const noexcept
{
    return _positionNdsThreshold;
}

void AssociationConfig::validate() const
{
    if (!std::isfinite(_positionNdsThreshold) || _positionNdsThreshold < 0.0)
    {
        throw std::invalid_argument("Invalid association configuration");
    }
}

AssociationConfig AssociationConfig::Builder::build() const
{
    if (!_maxAssociationSpeed || !_maxInitialDopplerDifference || !_positionNdsThreshold)
    {
        throw std::logic_error("AssociationConfig::Builder requires every value");
    }

    return AssociationConfig(
        *_maxAssociationSpeed,
        *_maxInitialDopplerDifference,
        *_positionNdsThreshold
    );
}

} // namespace target
