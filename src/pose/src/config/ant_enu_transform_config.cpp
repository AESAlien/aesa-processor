#include <pose/config/ant_enu_transform_config.hpp>

#include <stdexcept>
#include <utility>

namespace pose
{

AntEnuTransformConfig::AntEnuTransformConfig(math::SquareMatrix mountAntToBodyRotation)
    : _mountAntToBodyRotation(std::move(mountAntToBodyRotation))
{
    validate();
}

const math::SquareMatrix& AntEnuTransformConfig::mountAntToBodyRotation() const noexcept
{
    return _mountAntToBodyRotation;
}

void AntEnuTransformConfig::validate() const
{
    if (_mountAntToBodyRotation.size() != 3 || !_mountAntToBodyRotation.isRotation())
    {
        throw std::invalid_argument(
            "AntEnuTransformConfig: mountAntToBodyRotation must be a 3x3 rotation matrix"
        );
    }
}

AntEnuTransformConfig::Builder& AntEnuTransformConfig::Builder::mountAntToBodyRotation(
    math::SquareMatrix value
)
{
    _mountAntToBodyRotation.emplace(std::move(value));
    return *this;
}

AntEnuTransformConfig AntEnuTransformConfig::Builder::build() const
{
    if (!_mountAntToBodyRotation)
    {
        throw std::logic_error("AntEnuTransformConfig::Builder requires every value");
    }
    return AntEnuTransformConfig(*_mountAntToBodyRotation);
}

} // namespace pose
