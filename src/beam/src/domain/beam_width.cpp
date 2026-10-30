#include <beam/domain/beam_width.hpp>

namespace beam
{

BeamWidth::BeamWidth(
    math::Angle azimuth,
    math::Angle elevation
) : BeamWidth(math::AngularWidth(azimuth), math::AngularWidth(elevation))
{
}

BeamWidth::BeamWidth(
    math::AngularWidth azimuth,
    math::AngularWidth elevation
) : azimuth(azimuth),
    elevation(elevation)
{
}

bool BeamWidth::operator==(const BeamWidth& other) const noexcept
{
    return azimuth == other.azimuth && elevation == other.elevation;
}

bool BeamWidth::operator!=(const BeamWidth& other) const noexcept
{
    return !(*this == other);
}

}
