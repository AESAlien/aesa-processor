#pragma once

#include "dto/tick_event.hpp"
#include <pose/dto/set_enu_geo_transform_command.hpp>
#include <target/dto/detection_event.hpp>
#include <variant>

namespace app
{
using TargetInput = std::variant<
    target::DetectionEvent,
    pose::SetEnuGeoTransformCommand,
    TickEvent
>;
} // namespace app
