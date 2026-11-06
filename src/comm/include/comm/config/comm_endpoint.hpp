#pragma once

#include <cstdint>
#include <string>

namespace comm
{

class CommEndpoint
{
public:
    // Throws std::invalid_argument.
    CommEndpoint(std::string address, std::uint16_t port);

    const std::string& getAddress() const noexcept;

    // The value passed to bind() and compared with accepted addresses.
    std::uint32_t getIntAddress() const noexcept;

    std::uint16_t getPort() const noexcept;

    bool operator==(const CommEndpoint& other) const noexcept;

private:
    std::string _address;
    std::uint32_t _intAddress;
    std::uint16_t _port;
};

} // namespace comm
