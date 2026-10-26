#include "config_loader.hpp"

#include "env_file_reader.hpp"

#include "config_assembler.hpp"
#include "config_value_reader.hpp"

namespace app
{

Config ConfigLoader::load(const std::filesystem::path& path)
{
    const EnvFileReader envFileReader(path);
    const auto configValues = envFileReader.read();
    ConfigValueReader configValueReader(configValues);
    ConfigAssembler configAssembler(configValueReader);
    return configAssembler.assemble();
}

} // namespace app
