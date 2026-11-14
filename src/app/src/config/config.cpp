#include "config.hpp"

#include <utility>

namespace app
{

Config::Config(
    pose::PoseConfig poseConfig,
    beam::BeamConfig beamConfig,
    target::TargetConfig targetConfig,
    comm::CommConfig commConfig)
    : poseConfig(std::move(poseConfig)),
      beamConfig(std::move(beamConfig)),
      targetConfig(std::move(targetConfig)),
      commConfig(std::move(commConfig))
{
}

} // namespace app
