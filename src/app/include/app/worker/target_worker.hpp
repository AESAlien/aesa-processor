#pragma once

#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <app/channel/target_channel.hpp>
#include <target/controller/target_controller.hpp>
#include <app/worker/worker_error.hpp>

namespace aesa_processor::app
{
class TargetWorker
{
public:
    TargetWorker(
        TargetChannel& targetChannel,
        BeamChannel& beamChannel,
        CommChannel& commChannel,
        aesa_processor::target::TargetController& controller);

    void run();

private:
    TargetChannel& _targetChannel;
    BeamChannel& _beamChannel;
    CommChannel& _commChannel;
    aesa_processor::target::TargetController& _controller;
};
} // namespace aesa_processor::app