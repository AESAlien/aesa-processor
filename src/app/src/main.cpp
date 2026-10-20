#include <app/channel/beam_channel.hpp>
#include <app/channel/comm_channel.hpp>
#include <app/channel/target_channel.hpp>
#include <app/worker/beam_worker.hpp>
#include <app/worker/comm_worker.hpp>
#include <app/worker/target_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <beam/controller/beam_controller.hpp>
#include <comm/controller/comm_controller.hpp>
#include <target/config/target_config.hpp>
#include <target/controller/target_controller.hpp>

#include <array>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>
#include <utility>

namespace
{

constexpr std::chrono::milliseconds BEAM_PERIOD{10};

comm::CommConfiguration makeConfiguration()
{
    // 외부 프로그램은 자신의 source IP/port를 아래 endpoint에 맞춰 bind해야 한다.
    return
    {
        {"192.168.10.20", 3200},  // 이 프로그램의 TCP 서버
        {"192.168.10.10", 3100},  // 운용 콘솔
        {"192.168.10.100", 3300}  // 모의기
    };
}

template <typename Action>
void runWorker(const char* name, Action action)
{
    try
    {
        action();
        app::reportWorkerError(name, "Worker returned unexpectedly");
    }
    catch (const std::exception& error)
    {
        app::reportWorkerError(name, error.what());
    }
    catch (...)
    {
        app::reportWorkerError(name, "Unknown worker failure");
    }

    // 일반 모듈 예외는 기존 워커 내부에서 처리한다.
    // 여기까지 나온 예외/반환은 워커 자체가 끝난 상황이다.
    // 수신 취소 API가 없는 상태에서 join으로 멈추지 않도록 실패 종료한다.
    std::_Exit(EXIT_FAILURE);
}

} // namespace

int main()
{
    // 스레드 생성 도중 실패해도 joinable thread 소멸로 terminate되지 않게
    // catch 바깥에 스레드 객체를 둔다.
    std::array<std::thread, 5> threads;

    try
    {
        // 1. 채널 3개. CommChannel 내부에는 목적지별 큐 2개가 있다.
        app::BeamChannel beamChannel;
        app::TargetChannel targetChannel;
        app::CommChannel commChannel;

        // 2. 실제 구현된 컨트롤러.
        beam::BeamController beamController;
        target::TargetController targetController(target::TargetConfig{});
        comm::CommController commController(makeConfiguration());

        // 다른 스레드가 컨트롤러를 사용하기 전에 서버를 연다.
        commController.openServer();

        // 3. 역할별 워커 5개. Comm 워커 셋은 같은 컨트롤러/채널을 공유한다.
        app::BeamWorker beamWorker(beamChannel, commChannel, beamController);
        app::TargetWorker targetWorker(
            targetChannel, beamChannel, commChannel, targetController);
        app::CommWorker receiveWorker(
            commChannel, beamChannel, targetChannel, commController);
        app::CommWorker simulatorSendWorker(
            commChannel, beamChannel, targetChannel, commController);
        app::CommWorker consoleSendWorker(
            commChannel, beamChannel, targetChannel, commController);

        // 스레드를 시작한 뒤의 실패는 아래 내부 catch에서 즉시 처리한다.
        // 워커가 참조하는 채널/컨트롤러가 먼저 소멸하지 않게 한다.
        try
        {
            // 4. 각 워커를 별도 std::thread에서 실행한다.
            threads[0] = std::thread([&]
            {
                runWorker("BeamWorker", [&] { beamWorker.run(); });
            });

            threads[1] = std::thread([&]
            {
                runWorker("TargetWorker", [&] { targetWorker.run(); });
            });

            threads[2] = std::thread([&]
            {
                runWorker("CommReceive", [&] { receiveWorker.runReceive(); });
            });

            threads[3] = std::thread([&]
            {
                runWorker("CommSendSimulator", [&] { simulatorSendWorker.runSendSimulator(); });
            });

            threads[4] = std::thread([&]
            {
                runWorker("CommSendConsole", [&] { consoleSendWorker.runSendConsole(); });
            });

            std::cout
                << "[START] 5 worker threads running; beam tick = 10ms\n"
                << "[SERVER] 192.168.10.20:3200\n"
                << "[CONSOLE SOURCE] 192.168.10.10:3100\n"
                << "[SIMULATOR SOURCE] 192.168.10.100:3300\n"
                << "Send ON, then attitude, then detection messages.\n"
                << "Ctrl+C terminates the process; graceful receive shutdown is not implemented."
                << std::endl;

            // 5. main 스레드는 빔 생성 이벤트만 넣는다.
            // 컨트롤러 메서드 호출과 운용 ON/OFF 판단은 기존 워커/모듈에 맡긴다.
            const auto started = std::chrono::steady_clock::now();
            auto nextTick = started;

            while (true)
            {
                std::this_thread::sleep_until(nextTick);

                const auto now = std::chrono::steady_clock::now();
                const auto currentTime =
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - started);

                beamChannel.write(app::NextBeamCommandEvent{currentTime});

                nextTick += BEAM_PERIOD;
                if (nextTick <= now)
                {
                    // main이 지연됐을 때 과거 tick을 한꺼번에 쌓지 않는다.
                    nextTick = now + BEAM_PERIOD;
                }
            }
        }
        catch (const std::exception& error)
        {
            app::reportWorkerError("main", error.what());
            std::_Exit(EXIT_FAILURE);
        }
        catch (...)
        {
            app::reportWorkerError("main", "Unknown thread/timer failure");
            std::_Exit(EXIT_FAILURE);
        }
    }
    catch (const std::exception& error)
    {
        app::reportWorkerError("main", error.what());
    }
    catch (...)
    {
        app::reportWorkerError("main", "Unknown startup/timer failure");
    }

    // 여기로 오는 것은 스레드 생성 전의 초기화 실패다.
    return EXIT_FAILURE;
}
