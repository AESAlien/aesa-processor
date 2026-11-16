#pragma once

#include <pose/domain/geo_position.hpp>

#include <math/geometry/cartesian_position.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/linear_algebra/vector.hpp>

namespace pose
{

class EnuGeoTransform
{
public:
    // 원점과 결과의 고도는 WGS 84 타원체고다.
    explicit EnuGeoTransform(const GeoPosition& origin);

    // position_enu의 x, y, z는 원점 기준 동쪽, 북쪽, 위쪽 변위다.
    GeoPosition enuToGeo(const math::CartesianPosition& position_enu) const;
    math::CartesianPosition geoToEnu(const GeoPosition& position) const;

private:
    GeoPosition _origin;
    math::Vector _origin_ecef;
    math::SquareMatrix _rotationMatrix;
};

} // namespace pose
