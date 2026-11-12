#pragma once

#include "channel/beam_channel.hpp"
#include "channel/simulator_channel.hpp"
#include <beam/controller/beam_controller.hpp>
#include "worker/worker_error.hpp"

namespace app
{
class BeamWorker
{
public:
    BeamWorker(
        BeamChannel& beamChannel,
        SimulatorChannel& simulatorChannel,
        beam::BeamController& controller);

    void run();

private:
    BeamChannel& _beamChannel;
    SimulatorChannel& _simulatorChannel;
    beam::BeamController& _controller;
};
} // namespace app