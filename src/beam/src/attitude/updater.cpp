#include <beam/updater.hpp>
#include "rotation.hpp"
#include <cmath>

namespace beam
{

namespace
{
bool allFinite(const AttitudeDto& a)
{
    return std::isfinite(a.radar_lat_deg) && std::isfinite(a.radar_lon_deg)
        && std::isfinite(a.radar_alt_km) && std::isfinite(a.roll_deg)
        && std::isfinite(a.pitch_deg) && std::isfinite(a.yaw_deg);
}
}   // namespace

AttUpdateResult makeAttTransform(const AttitudeDto&, const AttitudeConfig&, 
    AttTransformDto& out)
{

}

}   // namespace beam