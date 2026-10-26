#include "config.hpp"

#include <utility>

namespace app
{

Config::Config(
    beam::BeamConfig beamConfig,
    target::TargetConfig targetConfig,
    comm::CommConfig commConfig)
    : beamConfig(std::move(beamConfig)),
      targetConfig(std::move(targetConfig)),
      commConfig(std::move(commConfig))
{
}

} // namespace app
