#pragma once

#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <beam/controller/beam_controller.hpp>
#include <app/worker/worker_error.hpp>

namespace app
{
class BeamWorker
{
public:
    BeamWorker(
        BeamChannel& beamChannel,
        CommChannel& commChannel,
        beam::BeamController& controller);

    void run();

private:
    BeamChannel& _beamChannel;
    CommChannel& _commChannel;
    beam::BeamController& _controller;
};
} // namespace app