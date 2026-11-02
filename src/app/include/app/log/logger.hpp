#pragma once

#include <cstddef>
#include <string>

namespace logging
{

void initialize(
    const std::string& path = "logs/radar.log",
    std::size_t queueSize = 4096);

void shutdown();

} // namespace logging