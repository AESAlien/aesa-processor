#include "config.hpp"

#include <utility>

namespace app
{

Config::Config(
    beam::BeamConfig beamConfig,
    target::TargetConfig targetConfig,
    comm::CommConfiguration commConfiguration)
    : beamConfig(std::move(beamConfig)),
      targetConfig(std::move(targetConfig)),
      commConfiguration(std::move(commConfiguration))
{
}

} // namespace app
