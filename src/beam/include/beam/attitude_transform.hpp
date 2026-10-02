#pragma once
#include <beam/dto/radar_attitude.hpp>
#include <beam/dto/antenna_to_enu.hpp>
#include <math/matrix.hpp>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace beam
{

struct AttitudeConfig
{
    double max_abs_pitch_deg = 89.0;
    math::Matrix mount_ant_to_body = math::Matrix::Identity(3);
    double mount_ortho_tol = 1e-6;
};

enum class AttitudeErrorCode : std::uint8_t { NotFinite, OutOfRange, GimbalLock, InvalidMount };

class AttitudeError : public std::invalid_argument
{
public:
    AttitudeError(AttitudeErrorCode code, const std::string& what)
        : std::invalid_argument(what), code_(code) {}

    AttitudeErrorCode code() const noexcept { return code_; }

private:
    AttitudeErrorCode code_;
};

// 실패 시 AttitudeError 를 던진다
AntennaToEnu makeAntennaToEnu(const RadarAttitude&, const AttitudeConfig&);

}   // namespace beam