#pragma once

#include <app/channel/beam_channel.hpp>
#include <app/channel/simulator_channel.hpp>
#include <beam/controller/beam_controller.hpp>
#include <app/worker/worker_error.hpp>

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