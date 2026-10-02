#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace beam
{

enum class AttitudeErrorCode : std::uint8_t
{
    NotFinite,
    OutOfRange,
    GimbalLock,
    InvalidMount
};

class AttitudeError : public std::invalid_argument
{
public:
    AttitudeError(AttitudeErrorCode code, const std::string& message)
        : std::invalid_argument(message), code_(code)
    {
    }

    AttitudeErrorCode code() const noexcept
    {
        return code_;
    }

private:
    AttitudeErrorCode code_;
};

}   // namespace beam
