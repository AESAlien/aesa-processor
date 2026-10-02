#include <beam/transform.hpp>
#include <math/angle.hpp>
#include <cmath>

namespace beam
{

namespace
{
inline double clamp1(double x) { return std::fmax(-1.0, std::fmin(1.0, x)); }

inline double wrap360(double d)
{
    d = std::fmod(d, 360.0);
    return ((d < 0.0) ? (d + 360.0) : d);
}

inline void aedToVec(double az_deg, double el_deg, double v[3])
{
    const double az = math::DegToRad(az_deg);
    const double el = math::DegToRad(el_deg);
    v[0] = std::cos(el) * std::sin(az);
    v[1] = std::cos(el) * std::cos(az);
    v[2] = std::sin(el);
}

inline void vecToAed(const double v[3], double& az_deg, double& el_deg)
{
    az_deg = wrap360(math::RadToDeg(std::atan2(v[0], v[1])));
    el_deg = math::RadToDeg(std::asin(clamp1(v[2])));
}
}   // namespace


bool antToEnuAngle(const AttTransformDto& xform,
    double az_ant_deg,  double el_ant_deg,
    double& az_enu_deg, double& el_enu_deg)
{
    if(!xform.valid) { return false; }

    double a[3], e[3];
    aedToVec(az_ant_deg, el_ant_deg, a);

    const auto& R = xform.rot_ant_to_enu;
    e[0] = R[0]*a[0] + R[1]*a[1] + R[2]*a[2];
    e[1] = R[3]*a[0] + R[4]*a[1] + R[5]*a[2];
    e[2] = R[6]*a[0] + R[7]*a[1] + R[8]*a[2];

    vecToAed(e, az_enu_deg, el_enu_deg);
    return true;
}

bool enuToAntAngle(const AttTransformDto& xform,
    double az_enu_deg,  double el_enu_deg,
    double& az_ant_deg, double& el_ant_deg)
{
    if(!xform.valid) { return false; }

    double e[3], a[3];
    aedToVec(az_enu_deg, el_enu_deg, e);

    const auto& R = xform.rot_ant_to_enu;
    a[0] = R[0]*e[0] + R[3]*e[1] + R[6]*e[2];
    a[1] = R[1]*e[0] + R[4]*e[1] + R[7]*e[2];
    a[2] = R[2]*e[0] + R[5]*e[1] + R[8]*e[2];

    vecToAed(a, az_ant_deg, el_ant_deg);
    return true;
}

}   // namespace beam
