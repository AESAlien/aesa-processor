#pragma once

#include <app/channel/beam_channel.hpp>
#include <app/channel/console_channel.hpp>
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
        ConsoleChannel& consoleChannel,
        target::TargetController& controller);

    void run();

private:
    TargetChannel& _targetChannel;
    BeamChannel& _beamChannel;
    ConsoleChannel& _consoleChannel;
    target::TargetController& _controller;
};
} // namespace app