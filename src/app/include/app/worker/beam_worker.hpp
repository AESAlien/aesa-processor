#pragma once

#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <beam/controller/beam_controller.hpp>
#include <app/worker/worker_error.hpp>

namespace aesa_processor::app
{
class BeamWorker
{
public:
    BeamWorker(
        BeamChannel& beamChannel,
        CommChannel& commChannel,
        aesa_processor::beam::BeamController& controller);

    void run();

private:
    BeamChannel& _beamChannel;
    CommChannel& _commChannel;
    aesa_processor::beam::BeamController& _controller;
};
} // namespace aesa_processor::app