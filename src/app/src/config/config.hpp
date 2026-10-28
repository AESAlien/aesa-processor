#pragma once

#include <beam/config/beam_config.hpp>
#include <comm/config/comm_config.hpp>
#include <target/config/target_config.hpp>

namespace app
{

struct Config
{
    Config(
        beam::BeamConfig beamConfig,
        target::TargetConfig targetConfig,
        comm::CommConfig commConfig);

    beam::BeamConfig beamConfig;
    target::TargetConfig targetConfig;
    comm::CommConfig commConfig;
};

} // namespace app
