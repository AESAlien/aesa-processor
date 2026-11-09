#include "detection_event_csv_logger.hpp"

#include <target/dto/detection_event.hpp>

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

#include <cstddef>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace app
{
namespace
{
std::string format(const target::DetectionEvent& event)
{
    const auto beamType = magic_enum::enum_name(event.beamType);
    if (beamType.empty())
    {
        throw std::invalid_argument("Unsupported detection beam type");
    }

    std::ostringstream prefix;
    prefix.imbue(std::locale::classic());
    prefix << beamType << ',' << event.requestId << ','
           << event.transmitTime.count() << ',' << event.plots.size();

    if (event.plots.empty())
    {
        return prefix.str() + ",,,,,,";
    }

    std::ostringstream rows;
    rows.imbue(std::locale::classic());
    rows << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (std::size_t index = 0; index < event.plots.size(); ++index)
    {
        if (index != 0)
        {
            rows << '\n';
        }
        const auto& plot = event.plots[index];
        rows << prefix.str() << ',' << index
             << ',' << plot.slantRange.m() << ',' << plot.azimuth.deg()
             << ',' << plot.elevation.deg() << ',' << plot.dopplerVelocity.mps()
             << ',' << plot.power_db;
    }
    return rows.str();
}
}

std::string DetectionEventCsvLogger::header()
{
    return "beamType,requestId,transmitTime_ms,plotCount,"
           "plotIndex,slantRange_m,azimuth_deg,elevation_deg,dopplerVelocity_mps,power_db";
}

void DetectionEventCsvLogger::log(const target::DetectionEvent& event) noexcept
{
    try
    {
        if (const auto logger = spdlog::get(LOGGER_NAME))
        {
            // 한 이벤트의 모든 플롯을 한 번에 전달해 비동기 큐에서도 행을 연속으로 기록한다.
            logger->info("{}", format(event));
        }
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "[DetectionEventCsvLogger] %s\n", error.what());
    }
    catch (...)
    {
        std::fprintf(stderr, "[DetectionEventCsvLogger] CSV logging failed\n");
    }
}
}
