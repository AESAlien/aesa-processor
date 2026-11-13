#pragma once

#include "dto/tick_event.hpp"
#include <target/dto/detection_event.hpp>
#include <variant>

namespace app
{
using TargetInput = std::variant<
    target::DetectionEvent,
    TickEvent
>;
} // namespace app
