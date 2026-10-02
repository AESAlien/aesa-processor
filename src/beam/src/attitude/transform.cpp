#include <beam/transform.hpp>
#include <math/angle.hpp>
#include <math/matrix.hpp>
#include <cmath>
#include <stdexcept>

namespace beam
{
namespace
{
inline double clamp1(double x) { return std::fmax(-1.0, std::fmin(1.0, x)); }

inline double wrap180(double d)
{
    d = std::fmod(d + 180.0, 360.0);
    if(d < 0.0) { d += 360.0; }
    return d - 180.0;
}

// 방위/고각 -> 3x1 단위 열벡터
inline math::Matrix aedToVec(double az_deg, double el_deg)
{
    const double az = math::DegToRad(az_deg);
    const double el = math::DegToRad(el_deg);
    return math::Matrix{
        { std::cos(el) * std::sin(az) },
        { std::cos(el) * std::cos(az) },
        { std::sin(el) }
    };
}

// 3x1 열벡터 -> 방위/고각
inline void vecToAed(const math::Matrix& v, double& az_deg, double& el_deg)
{
    az_deg = wrap180(math::RadToDeg(std::atan2(v(0,0), v(1,0))));
    el_deg = math::RadToDeg(std::asin(clamp1(v(2,0))));
}
}   // namespace


void antToEnuAngle(const AntennaToEnu& xform,
    double az_ant_deg,  double el_ant_deg,
    double& az_enu_deg, double& el_enu_deg)
{
    if(!xform.valid) { throw std::invalid_argument("AntennaToEnu is not valid"); }

    const math::Matrix e = xform.rot_ant_to_enu * aedToVec(az_ant_deg, el_ant_deg);

    vecToAed(e, az_enu_deg, el_enu_deg);
}

void enuToAntAngle(const AntennaToEnu& xform,
    double az_enu_deg,  double el_enu_deg,
    double& az_ant_deg, double& el_ant_deg)
{
    if(!xform.valid) { throw std::invalid_argument("AntennaToEnu is not valid"); }

    const math::Matrix a = xform.rot_ant_to_enu.Transpose() * aedToVec(az_enu_deg, el_enu_deg);

    vecToAed(a, az_ant_deg, el_ant_deg);
}

}   // namespace beam
