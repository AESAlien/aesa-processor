#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>

namespace app
{

class EnvFileReader final
{
public:
    explicit EnvFileReader(std::filesystem::path path);

    std::unordered_map<std::string, std::string> read() const;

private:
    const std::filesystem::path _path;
};

} // namespace app
