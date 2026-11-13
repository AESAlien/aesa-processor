#pragma once

#include <beam/dto/set_attitude_command.hpp>
#include <target/dto/track_event.hpp>
#include <variant>
#include <vector>

namespace app
{
using ConsoleInput = std::variant<
    beam::SetAttitudeCommand,
    std::vector<target::TrackEvent>
>;
} // namespace app
