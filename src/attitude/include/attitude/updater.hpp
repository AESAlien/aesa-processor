#pragma once
#include <attitude/dto/attitude_dto.hpp>
#include <attitude/dto/attTransform_dto.hpp>
#include <attitude/port/attitude_source.hpp>
#include <chrono>
#include <cstdint>
#include <mutex>

namespace attitude
{

struct AttitudeConfig
{
    std::int64_t max_age_ms = 200;
    double max_abs_pitch_deg = 89.0;
    std::array<double,9> mount_ant_to_body{1,0,0, 0,1,0, 0,0,1};
};

enum class AttUpdateResult : std::uint8_t { Ok, NotFinite, OutOfRange, GimbalLock };

class AttitudeStore
{
public:
    explicit AttitudeStore(AttitudeConfig cfg = {}) : cfg_(cfg) {}
    AttUpdateResult update(const AttitudeDto& att, std::int64_t rx_time_ns);
    AttTransformDto snapshot(std::int64_t now_ns) const;
    std::uint32_t rejectCount(void) const;
private:
    AttitudeConfig      cfg_;
    mutable std::mutex  mtx_;
    AttTransformDto     latest_{};
    std::uint32_t       reject_count_ = 0;
};

AttQueStatus pollOnce(AttitudePort& port, AttitudeStore& store,
                    std::chrono::milliseconds timeout,
                    AttUpdateResult* out_res = nullptr);

inline std::int64_t nowNs(void)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

}   // namespace attitude