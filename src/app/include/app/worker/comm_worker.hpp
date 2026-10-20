#pragma once

#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <app/channel/target_channel.hpp>
#include <comm/controller/comm_controller.hpp>
#include <app/worker/worker_error.hpp>

namespace aesa_processor::app
{
class CommWorker
{
public:
    CommWorker(
        CommChannel& commChannel,
        BeamChannel& beamChannel,
        TargetChannel& targetChannel,
        aesa_processor::comm::CommController& controller);

    // 이 버전에서는 receiveData()의 nullopt를 수신 종료로 해석한다.
    void runReceive();

    void runSendSimulator();

    void runSendConsole();

private:
    CommChannel& _commChannel;
    BeamChannel& _beamChannel;
    TargetChannel& _targetChannel;
    aesa_processor::comm::CommController& _controller;
};
} // namespace aesa_processor::app