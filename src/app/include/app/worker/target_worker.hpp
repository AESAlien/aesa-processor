#pragma once

#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <app/channel/target_channel.hpp>
#include <target/controller/target_controller.hpp>
#include <app/worker/worker_error.hpp>

namespace app
{
class TargetWorker
{
public:
    TargetWorker(
        TargetChannel& targetChannel,
        BeamChannel& beamChannel,
        CommChannel& commChannel,
        target::TargetController& controller);

    void run();

private:
    TargetChannel& _targetChannel;
    BeamChannel& _beamChannel;
    CommChannel& _commChannel;
    target::TargetController& _controller;
};
} // namespace app