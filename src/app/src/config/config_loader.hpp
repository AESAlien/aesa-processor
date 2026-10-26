#pragma once

#include "config.hpp"

#include <filesystem>

namespace app
{

class ConfigLoader final
{
public:
    static Config load(const std::filesystem::path& path);
};

} // namespace app
