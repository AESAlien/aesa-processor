#include <comm/config/comm_endpoint.hpp>

#include <array>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace comm
{
namespace
{

constexpr std::size_t OCTET_COUNT = 4;

// Throws std::invalid_argument.
std::uint32_t parseIpv4Address(const std::string& address)
{
    const std::invalid_argument invalidAddress(
        "Comm endpoint requires a valid IPv4 address");

    std::array<std::uint8_t, OCTET_COUNT> octets{};
    std::size_t partCount = 0;
    std::size_t start = 0;
    while (start <= address.size())
    {
        const std::size_t separator = address.find('.', start);
        const std::size_t length = separator == std::string::npos
            ? std::string::npos
            : separator - start;
        const std::string part = address.substr(start, length);
        if (partCount == OCTET_COUNT || part.empty() || part.size() > 3 ||
            (part.size() > 1 && part.front() == '0'))
        {
            throw invalidAddress;
        }

        unsigned int value = 0;
        for (char character : part)
        {
            if (character < '0' || character > '9')
            {
                throw invalidAddress;
            }
            value = value * 10 + static_cast<unsigned int>(character - '0');
        }
        if (value > 255)
        {
            throw invalidAddress;
        }

        octets[partCount] = static_cast<std::uint8_t>(value);
        ++partCount;
        if (separator == std::string::npos)
        {
            break;
        }
        start = separator + 1;
    }

    if (partCount != OCTET_COUNT)
    {
        throw invalidAddress;
    }

    // The socket API needs the octets in this order in memory,
    // whatever the byte order of this machine.
    std::uint32_t intAddress = 0;
    std::memcpy(&intAddress, octets.data(), octets.size());
    return intAddress;
}

} // namespace

CommEndpoint::CommEndpoint(std::string address, std::uint16_t port)
    : _address(std::move(address)),
      _intAddress(parseIpv4Address(_address)),
      _port(port)
{
    if (_port == 0)
    {
        throw std::invalid_argument("Comm endpoint requires a port");
    }
}

const std::string& CommEndpoint::getAddress() const noexcept
{
    return _address;
}

std::uint32_t CommEndpoint::getIntAddress() const noexcept
{
    return _intAddress;
}

std::uint16_t CommEndpoint::getPort() const noexcept
{
    return _port;
}

bool CommEndpoint::operator==(const CommEndpoint& other) const noexcept
{
    return _intAddress == other._intAddress && _port == other._port;
}

} // namespace comm
