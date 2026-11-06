#include <target/config/association_config.hpp>

#include <cmath>
#include <stdexcept>

namespace target
{

using namespace math::literals;

AssociationConfig::AssociationConfig(
    math::LinearVelocity maxAssociationSpeed,
    math::LinearVelocity maxInitialDopplerDifference,
    double positionNdsThreshold
) : _maxAssociationSpeed(maxAssociationSpeed),
    _maxInitialDopplerDifference(maxInitialDopplerDifference),
    _positionNdsThreshold(positionNdsThreshold)
{
    validate();
}

const math::LinearVelocity& AssociationConfig::maxAssociationSpeed() const noexcept
{
    return _maxAssociationSpeed;
}

const math::LinearVelocity& AssociationConfig::maxInitialDopplerDifference() const noexcept
{
    return _maxInitialDopplerDifference;
}

double AssociationConfig::positionNdsThreshold() const noexcept
{
    return _positionNdsThreshold;
}

void AssociationConfig::validate() const
{
    if (_maxAssociationSpeed.lessThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE) ||
        _maxInitialDopplerDifference.lessThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE) ||
        !std::isfinite(_positionNdsThreshold) || _positionNdsThreshold < 0.0)
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
        *_positionNdsThreshold);
}

} // namespace target
