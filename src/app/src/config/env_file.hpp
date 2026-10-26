#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>

namespace app
{

class EnvFile final
{
public:
    explicit EnvFile(std::filesystem::path path);

    std::unordered_map<std::string, std::string> load() const;

private:
    const std::filesystem::path _path;
};

} // namespace app
