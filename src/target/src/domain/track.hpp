#pragma once

#include "domain/tentative_track.hpp"

#include <variant>

namespace target
{

using Track = std::variant<TentativeTrack, ConfirmedTrack>;

} // namespace target
