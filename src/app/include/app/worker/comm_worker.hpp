#pragma once

#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <app/channel/target_channel.hpp>
#include <comm/controller/comm_controller.hpp>
#include <app/worker/worker_error.hpp>

#include <atomic>

namespace app
{
class CommWorker
{
public:
    CommWorker(
        CommChannel& commChannel,
        BeamChannel& beamChannel,
        TargetChannel& targetChannel,
        comm::CommController& controller);

    void runReceive(const std::atomic_bool& stopRequested);
    void runSendSimulator();
    void runSendConsole();

private:
    CommChannel& _commChannel;
    BeamChannel& _beamChannel;
    TargetChannel& _targetChannel;
    comm::CommController& _controller;
};
} // namespace app