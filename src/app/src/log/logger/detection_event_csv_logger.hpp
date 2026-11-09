#pragma once

#include <target/dto/log/detection_event_logger.hpp>

#include <string>

namespace app
{
class DetectionEventCsvLogger final : public target::DetectionEventLogger
{
public:
    static constexpr char LOGGER_NAME[] = "DetectionEvent";

    static std::string header();
    void log(const target::DetectionEvent& event) noexcept override;
};
}
