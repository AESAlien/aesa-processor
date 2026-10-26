#pragma once

#include "config.hpp"

#include <filesystem>

namespace app
{

class ConfigFactory final
{
public:
    static Config create(const std::filesystem::path& path);
};

} // namespace app
