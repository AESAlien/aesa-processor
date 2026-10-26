#include "config_factory.hpp"

#include "env_file.hpp"

#include "config_assembler.hpp"
#include "config_value_reader.hpp"

namespace app
{

Config ConfigFactory::create(const std::filesystem::path& path)
{
    const EnvFile envFile(path);
    const auto configValues = envFile.load();
    ConfigValueReader configValueReader(configValues);
    ConfigAssembler configAssembler(configValueReader);
    return configAssembler.assemble();
}

} // namespace app
