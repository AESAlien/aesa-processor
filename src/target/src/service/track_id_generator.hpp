#pragma once

#include <cstdint>

namespace target
{

class TrackIdGenerator
{
public:
    std::uint32_t nextId() const;
    // Advance only after the object using nextId() has been successfully stored.
    void commit() noexcept;

private:
    std::uint32_t _nextId = 1;
};

} // namespace target
