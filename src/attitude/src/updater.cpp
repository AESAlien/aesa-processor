#include <attitude/updater.hpp>
#include "rotation.hpp"
#include <cmath>

namespace attitude
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

AttUpdateResult AttitudeStore::update(const AttitudeDto& a, std::int64_t rx_time_ns)
{
    AttUpdateResult res = AttUpdateResult::Ok;

    if(!allFinite(a)) {
        res = AttUpdateResult::NotFinite;
    } else if(a.radar_lat_deg < -90.0  ||  a.radar_lat_deg > 90.0  ||
              a.radar_lon_deg < -180.0 ||  a.radar_lon_deg > 180.0 ||
              a.radar_alt_km  < -0.5   ||  a.radar_alt_km  > 100.0 ||
              a.roll_deg      < -180.0 ||  a.roll_deg      > 180.0 ||
              a.yaw_deg       < -360.0 ||  a.yaw_deg       > 360.0) {
        res = AttUpdateResult::OutOfRange;
    } else if(std::fabs(a.pitch_deg) >= cfg_.max_abs_pitch_deg) {
        res = AttUpdateResult::GimbalLock;
    }

    AttTransformDto next{};
    if(res == AttUpdateResult::Ok) {
        const Mat3 R_body_enu = bodyToEnu(a.roll_deg, a.pitch_deg, a.yaw_deg);
        next.att            = a;
        next.rx_time_ns     = rx_time_ns;
        next.rot_ant_to_enu = mul(R_body_enu, cfg_.mount_ant_to_body);
        next.valid          = true;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    if(res != AttUpdateResult::Ok) {
        ++reject_count_;
        return res;
    }
    next.update_seq = latest_.update_seq + 1;
    latest_ = next;
    return res;
}

AttTransformDto AttitudeStore::snapshot(std::int64_t now_ns) const
{
    AttTransformDto out;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        out = latest_;
    }
    if(out.valid) {
        const std::int64_t age_ns = now_ns - out.rx_time_ns;
        if(age_ns > cfg_.max_age_ms * 1'000'000LL) { out.valid = false; }
    }
    return out;
}

std::uint32_t AttitudeStore::rejectCount(void) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return reject_count_;
}

AttQueStatus pollOnce(AttitudePort& port, AttitudeStore& store,
                      std::chrono::milliseconds timeout,
                      AttUpdateResult* out_res)
{
    AttitudeDto att{};
    std::int64_t rx_ns = 0;
    
    const AttQueStatus st = port.read(att, rx_ns, timeout);
    if(st == AttQueStatus::Success) {
        const AttUpdateResult r = store.update(att, rx_ns);
        if(out_res) { *out_res = r; }
    }
    return st;
}

}   // namespace attitude