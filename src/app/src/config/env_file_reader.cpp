#include "env_file_reader.hpp"

#include <cctype>
#include <cstddef>
#include <fstream>
#include <istream>
#include <stdexcept>
#include <string>
#include <utility>

namespace app
{

namespace
{

std::string trim(const std::string& value)
{
    std::size_t first = 0;
    while (first < value.size() &&
           std::isspace(static_cast<unsigned char>(value[first])) != 0)
    {
        ++first;
    }

    std::size_t last = value.size();
    while (last > first &&
           std::isspace(static_cast<unsigned char>(value[last - 1])) != 0)
    {
        --last;
    }

    return value.substr(first, last - first);
}

std::string parseValue(const std::string& text, std::size_t lineNumber)
{
    std::string value = trim(text);
    if (value.empty())
    {
        return value;
    }

    if (value.front() == '\'' || value.front() == '"')
    {
        const char QUOTE = value.front();
        if (value.size() < 2 || value.back() != QUOTE)
        {
            throw std::runtime_error(
                "Invalid quoted value at line " +
                std::to_string(lineNumber));
        }
        return value.substr(1, value.size() - 2);
    }

    for (std::size_t index = 0; index < value.size(); ++index)
    {
        if (value[index] == '#' &&
            (index == 0 ||
             std::isspace(static_cast<unsigned char>(value[index - 1])) != 0))
        {
            return trim(value.substr(0, index));
        }
    }

    return value;
}

std::unordered_map<std::string, std::string> parse(std::istream& input)
{
    std::unordered_map<std::string, std::string> variables;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line))
    {
        ++lineNumber;
        const std::string CONTENT = trim(line);
        if (CONTENT.empty() || CONTENT.front() == '#')
        {
            continue;
        }

        const std::size_t SEPARATOR = CONTENT.find('=');
        if (SEPARATOR == std::string::npos)
        {
            throw std::runtime_error(
                "Expected KEY=VALUE at line " +
                std::to_string(lineNumber));
        }

        const std::string KEY = trim(CONTENT.substr(0, SEPARATOR));
        if (KEY.empty())
        {
            throw std::runtime_error(
                "Configuration key is empty at line " +
                std::to_string(lineNumber));
        }

        const std::string VALUE = parseValue(
            CONTENT.substr(SEPARATOR + 1),
            lineNumber);
        const auto INSERTED = variables.emplace(KEY, VALUE);
        if (!INSERTED.second)
        {
            throw std::runtime_error(
                "Duplicate configuration key: " + KEY);
        }
    }

    if (input.bad())
    {
        throw std::runtime_error("Unable to read environment input");
    }

    return variables;
}

} // namespace

EnvFileReader::EnvFileReader(std::filesystem::path path)
    : _path(std::move(path))
{
}

std::unordered_map<std::string, std::string> EnvFileReader::read() const
{
    std::ifstream file(_path);
    if (!file.is_open())
    {
        throw std::runtime_error(
            "Unable to open configuration file: " +
            _path.string());
    }

    return parse(file);
}

} // namespace app
