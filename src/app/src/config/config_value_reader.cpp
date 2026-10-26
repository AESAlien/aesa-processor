#include "config_value_reader.hpp"

#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace app
{

ConfigValueReader::ConfigValueReader(
    std::unordered_map<std::string, std::string> configValues)
    : _configValues(std::move(configValues))
{
}

const std::string& ConfigValueReader::requireString(const char* key)
{
    const auto value = _configValues.find(key);
    if (value == _configValues.end())
    {
        throw std::runtime_error(
            "Missing required configuration key: " +
            std::string(key));
    }

    _consumedKeys.emplace(key);
    return value->second;
}

double ConfigValueReader::requireDouble(const char* key)
{
    return parseDouble(requireString(key), key);
}

double ConfigValueReader::parseDouble(
    const std::string& text,
    const char* key) const
{
    std::size_t parsedLength = 0;
    double value = 0.0;
    try
    {
        value = std::stod(text, &parsedLength);
    }
    catch (const std::exception&)
    {
        throw std::runtime_error(
            "Invalid floating-point value for " +
            std::string(key));
    }

    if (parsedLength != text.size() || !std::isfinite(value))
    {
        throw std::runtime_error(
            "Invalid floating-point value for " +
            std::string(key));
    }
    return value;
}

math::SquareMatrix ConfigValueReader::requireSquareMatrix(
    const char* key,
    std::size_t size)
{
    if (size != 0 && size > std::numeric_limits<std::size_t>::max() / size)
    {
        throw std::runtime_error(
            "Square matrix size is out of range for " +
            std::string(key));
    }

    const std::size_t expectedElementCount = size * size;
    std::istringstream stream(requireString(key));
    std::vector<double> elements;
    std::string element;
    while (std::getline(stream, element, ','))
    {
        std::size_t parsedLength = 0;
        try
        {
            const double value = std::stod(element, &parsedLength);
            if (parsedLength != element.size() || !std::isfinite(value))
            {
                throw std::invalid_argument("Invalid matrix element");
            }
            elements.push_back(value);
        }
        catch (const std::exception&)
        {
            throw std::runtime_error(
                "Invalid matrix value for " + std::string(key));
        }
    }

    if (elements.size() != expectedElementCount)
    {
        throw std::runtime_error(
            "Unexpected matrix element count for " + std::string(key));
    }

    math::Matrix matrix(size, size);
    for (std::size_t index = 0; index < elements.size(); ++index)
    {
        matrix(index / size, index % size) = elements[index];
    }
    return math::SquareMatrix(std::move(matrix));
}

unsigned long long ConfigValueReader::requireUnsignedValue(const char* key)
{
    const std::string& text = requireString(key);
    if (text.empty() || text.front() == '-')
    {
        throw std::runtime_error(
            "Invalid unsigned integer value for " +
            std::string(key));
    }

    std::size_t parsedLength = 0;
    unsigned long long value = 0;
    try
    {
        value = std::stoull(text, &parsedLength, 10);
    }
    catch (const std::exception&)
    {
        throw std::runtime_error(
            "Invalid unsigned integer value for " +
            std::string(key));
    }

    if (parsedLength != text.size())
    {
        throw std::runtime_error(
            "Invalid unsigned integer value for " +
            std::string(key));
    }
    return value;
}

void ConfigValueReader::ensureAllConsumed() const
{
    for (const auto& configValue : _configValues)
    {
        if (_consumedKeys.find(configValue.first) == _consumedKeys.end())
        {
            throw std::runtime_error(
                "Unknown configuration key: " +
                configValue.first);
        }
    }
}

} // namespace app
