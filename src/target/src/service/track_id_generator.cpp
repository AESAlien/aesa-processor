#include "track_id_generator.hpp"

#include <limits>
#include <stdexcept>

namespace target
{
namespace
{

constexpr std::uint32_t MAX_TRACK_ID = std::numeric_limits<std::uint16_t>::max();

} // namespace

std::uint32_t TrackIdGenerator::nextId() const
{
    if (_nextId > MAX_TRACK_ID)
    {
        throw std::overflow_error("Target ID exceeds the console ID range");
    }
    return _nextId;
}

void TrackIdGenerator::commit() noexcept
{
    if (_nextId <= MAX_TRACK_ID)
    {
        ++_nextId;
    }
}

} // namespace target
