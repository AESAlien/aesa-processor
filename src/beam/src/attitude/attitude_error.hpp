#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace beam
{

enum class AttitudeErrorCode : uint8_t
{
    NOT_FINITE,
    OUT_OF_RANGE,
    GIMBAL_LOCK,
    INVALID_MOUNT
};

class AttitudeError : public std::invalid_argument
{
public:
    AttitudeError(AttitudeErrorCode code, const std::string& message) : std::invalid_argument(message), _code(code)
    {
    }

    AttitudeErrorCode code() const noexcept
    {
        return _code;
    }

private:
    AttitudeErrorCode _code;
};

} // namespace beam
