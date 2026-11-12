#pragma once

#include "channel/beam_channel.hpp"
#include "channel/target_channel.hpp"
#include "channel/console_channel.hpp"
#include <comm/dto/incoming_dto.hpp>

#include <functional>
#include <optional>

namespace app
{
class CommReceiveWorker
{
public:
    using ReceiveData = std::function<std::optional<comm::IncomingDto>()>;

    CommReceiveWorker(
        BeamChannel& beamChannel,
        TargetChannel& targetChannel,
        ConsoleChannel& consoleChannel,
        ReceiveData receiveData
    );

    // 수신 함수가 nullopt를 반환하면 종료한다. 오류는 기록 후 재시도한다.
    void run();

private:
    BeamChannel& _beamChannel;
    TargetChannel& _targetChannel;
    ConsoleChannel& _consoleChannel;
    ReceiveData _receiveData;
};
} // namespace app
