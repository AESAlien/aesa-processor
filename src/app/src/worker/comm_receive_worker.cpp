#include <app/worker/comm_receive_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <chrono>
#include <exception>
#include <thread>
#include <utility>
#include <variant>

namespace app
{
CommReceiveWorker::CommReceiveWorker(
    BeamChannel& beamChannel,
    TargetChannel& targetChannel,
    ConsoleChannel& consoleChannel,
    ReceiveData receiveData
) : _beamChannel(beamChannel),
    _targetChannel(targetChannel),
    _consoleChannel(consoleChannel),
    _receiveData(std::move(receiveData))
{
}

void CommReceiveWorker::run()
{
    while (true)
    {
        try
        {
            auto incoming = _receiveData();

            if (!incoming)
            {
                return;
            }

            auto& message = *incoming;

            if (auto* attitude =
                    std::get_if<beam::SetAttitudeCommand>(&message))
            {
                // 모의기의 자세 정보는 빔 처리를 기다리지 않고 전시기에 바로 전달한다.
                _consoleChannel.write(*attitude);
                _beamChannel.write(std::move(*attitude));
            }
            else if (auto* state =
                         std::get_if<beam::SetOperationStateCommand>(&message))
            {
                _beamChannel.write(std::move(*state));
            }
            else
            {
                auto& detection =
                    std::get<target::DetectionEvent>(message);

                _targetChannel.write(std::move(detection));
            }

            continue;
        }
        catch (const std::exception& error)
        {
            reportWorkerError(
                "CommReceive",
                error.what());
        }
        catch (...)
        {
            reportWorkerError(
                "CommReceive",
                "Unknown exception");
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    }
}

} // namespace app
