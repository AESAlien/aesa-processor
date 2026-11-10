#pragma once

#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <beam/controller/beam_controller.hpp>
#include <app/worker/worker_error.hpp>

#include <memory>

namespace spdlog
{
class logger;
}

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
    // 처리 실패를 기록할 로거. 등록되지 않았으면 nullptr이며, 이때는 기록하지 않는다.
    std::shared_ptr<spdlog::logger> _logger;
};
} // namespace app