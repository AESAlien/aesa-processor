#include <pose/domain/enu_geo_transform.hpp>
#include <math/trigonometry.hpp>

#include <cmath>
#include <stdexcept>

namespace pose
{
namespace
{
constexpr double SEMI_MAJOR_AXIS_M = 6378137.0;
constexpr double FLATTENING = 1.0 / 298.257223563;
constexpr double ECCENTRICITY_SQUARED = FLATTENING * (2.0 - FLATTENING);
using namespace math::literals;

const math::Angle LATITUDE_TOLERANCE = 1e-14_rad;
constexpr int MAX_ITERATIONS = 16;

double primeVerticalRadius(double sinLatitude)
{
    return SEMI_MAJOR_AXIS_M / std::sqrt(1.0 - ECCENTRICITY_SQUARED * sinLatitude * sinLatitude);
}

math::Vector toEcef(const GeoPosition& position)
{
    const auto sinLatitude = math::sin(position.latitude);
    const auto cosLatitude = math::cos(position.latitude);
    const auto radius = primeVerticalRadius(sinLatitude);
    const auto horizontal = (radius + position.altitude.m()) * cosLatitude;
    return math::Vector({
        horizontal * math::cos(position.longitude),
        horizontal * math::sin(position.longitude),
        (radius * (1.0 - ECCENTRICITY_SQUARED) + position.altitude.m()) * sinLatitude
    });
}

math::SquareMatrix makeEnuToEcef(const GeoPosition& origin)
{
    const auto sinLatitude = math::sin(origin.latitude);
    const auto cosLatitude = math::cos(origin.latitude);
    const auto sinLongitude = math::sin(origin.longitude);
    const auto cosLongitude = math::cos(origin.longitude);

    return math::SquareMatrix({
        {-sinLongitude, -sinLatitude * cosLongitude, cosLatitude * cosLongitude},
        { cosLongitude, -sinLatitude * sinLongitude, cosLatitude * sinLongitude},
        {          0.0,                 cosLatitude,                sinLatitude}
    });
}
} // namespace

EnuGeoTransform::EnuGeoTransform(const GeoPosition& origin)
  : _origin(origin),
    _origin_ecef(toEcef(origin)),
    _rotationMatrix(makeEnuToEcef(origin))
{
}

GeoPosition EnuGeoTransform::enuToGeo(const math::CartesianPosition& position_enu) const
{
    if (position_enu.x == 0.0_m && position_enu.y == 0.0_m && position_enu.z == 0.0_m)
    {
        return _origin;
    }

    const auto displacement_enu = math::Vector({position_enu.x.m(), position_enu.y.m(), position_enu.z.m()});
    const auto ecef = _origin_ecef + _rotationMatrix * displacement_enu;
    const auto horizontal = std::hypot(ecef(0), ecef(1));
    const auto vertical = ecef(2);

    if (!std::isfinite(horizontal) || !std::isfinite(vertical) || (horizontal == 0.0 && vertical == 0.0))
    {
        throw std::invalid_argument("ECEF position has no geodetic coordinates");
    }

    const auto longitude = horizontal == 0.0
        ? _origin.longitude
        : math::CircularAngle(math::atan2(math::Length::fromMeters(ecef(1)), math::Length::fromMeters(ecef(0))));
    const auto horizontalDistance = math::Length::fromMeters(horizontal);
    const auto verticalDistance = math::Length::fromMeters(vertical);
    auto latitude = math::atan2(verticalDistance, horizontalDistance * (1.0 - ECCENTRICITY_SQUARED));

    for (auto iteration = 0; iteration < MAX_ITERATIONS; ++iteration)
    {
        const auto sinLatitude = math::sin(latitude);
        const auto correction = math::Length::fromMeters(
            ECCENTRICITY_SQUARED * primeVerticalRadius(sinLatitude) * sinLatitude
        );
        const auto next = math::atan2(verticalDistance + correction, horizontalDistance);
        
        if (math::abs(next - latitude).lessThanOrEqual(LATITUDE_TOLERANCE, math::Angle::NONE_TOLERANCE))
        {
            const auto sinNext = math::sin(next);
            const auto altitude = horizontal * math::cos(next) + vertical * sinNext
                - SEMI_MAJOR_AXIS_M * std::sqrt(1.0 - ECCENTRICITY_SQUARED * sinNext * sinNext);

            return GeoPosition(
                math::TiltAngle(next),
                longitude,
                math::Length::fromMeters(altitude)
            );
        }
        
        latitude = next;
    }
    throw std::domain_error("Geodetic latitude did not converge");
}

math::CartesianPosition EnuGeoTransform::geoToEnu(const GeoPosition& position) const
{
    const auto displacement_ecef = toEcef(position) - _origin_ecef;
    const auto position_enu = math::transpose(_rotationMatrix) * displacement_ecef;
    
    return math::CartesianPosition(
        math::Length::fromMeters(position_enu(0)),
        math::Length::fromMeters(position_enu(1)),
        math::Length::fromMeters(position_enu(2))
    );
}

} // namespace pose
