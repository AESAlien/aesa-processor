#pragma once

#include <cstdint>

namespace comm::port
{

class TrackDeletionSource
{
public:
    virtual ~TrackDeletionSource() = default;

    virtual bool read(std::uint16_t& trackId) = 0;
};

} // namespace comm::port
