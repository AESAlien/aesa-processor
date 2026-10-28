#include <comm/config/comm_config.hpp>

#include <stdexcept>
#include <string>
#include <utility>

namespace comm
{
namespace
{

bool isIpv4Address(const std::string& address)
{
    std::size_t partCount = 0;
    std::size_t start = 0;
    while (start <= address.size())
    {
        const std::size_t separator = address.find('.', start);
        const std::size_t length = separator == std::string::npos
            ? std::string::npos
            : separator - start;
        const std::string part = address.substr(start, length);
        if (part.empty() || part.size() > 3 ||
            (part.size() > 1 && part.front() == '0'))
        {
            return false;
        }

        unsigned int value = 0;
        for (char character : part)
        {
            if (character < '0' || character > '9')
            {
                return false;
            }
            value = value * 10 + static_cast<unsigned int>(character - '0');
        }
        if (value > 255)
        {
            return false;
        }

        ++partCount;
        if (separator == std::string::npos)
        {
            break;
        }
        start = separator + 1;
    }
    return partCount == 4;
}

} // namespace

CommEndpoint::CommEndpoint(std::string address, std::uint16_t port)
    : _address(std::move(address)),
      _port(port)
{
    validate();
}

const std::string& CommEndpoint::address() const noexcept
{
    return _address;
}

std::uint16_t CommEndpoint::port() const noexcept
{
    return _port;
}

void CommEndpoint::validate() const
{
    if (_address.empty() || _port == 0)
    {
        throw std::invalid_argument(
            "Comm endpoint requires both address and port");
    }
    if (!isIpv4Address(_address))
    {
        throw std::invalid_argument(
            "Comm endpoint requires a valid IPv4 address");
    }
}

CommConfig::CommConfig(
    CommEndpoint server,
    CommEndpoint operatorConsole,
    CommEndpoint scenarioSimulator)
    : _server(std::move(server)),
      _operatorConsole(std::move(operatorConsole)),
      _scenarioSimulator(std::move(scenarioSimulator))
{
    validate();
}

const CommEndpoint& CommConfig::server() const noexcept
{
    return _server;
}

const CommEndpoint& CommConfig::operatorConsole() const noexcept
{
    return _operatorConsole;
}

const CommEndpoint& CommConfig::scenarioSimulator() const noexcept
{
    return _scenarioSimulator;
}

void CommConfig::validate() const
{
    if (_operatorConsole.address() == _scenarioSimulator.address() &&
        _operatorConsole.port() == _scenarioSimulator.port())
    {
        throw std::invalid_argument(
            "Communication peer endpoints must be distinct");
    }
}

} // namespace comm
