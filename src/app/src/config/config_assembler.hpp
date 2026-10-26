#pragma once

#include "config_value_reader.hpp"

#include "config.hpp"

#include <beam/config/beam_config.hpp>
#include <comm/config/comm_config.hpp>
#include <target/config/target_config.hpp>

namespace app
{

class ConfigAssembler final
{
public:
    explicit ConfigAssembler(ConfigValueReader values);

    Config assemble();

private:
    beam::BeamConfig createBeamConfig();
    target::TargetConfig createTargetConfig();
    comm::CommConfiguration createCommConfig();

    ConfigValueReader _reader;
};

} // namespace app
