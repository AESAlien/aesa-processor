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
        : std::invalid_argument(message), _code(code)
    {
    }

    AttitudeErrorCode Code() const noexcept
    {
        return _code;
    }

private:
    AttitudeErrorCode _code;
};

} // namespace beam
