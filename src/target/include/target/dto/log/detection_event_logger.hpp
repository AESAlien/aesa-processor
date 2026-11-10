#pragma once

namespace target
{
struct DetectionEvent;

class DetectionEventLogger
{
public:
    virtual ~DetectionEventLogger() = default;
    virtual void log(const DetectionEvent& detection) noexcept = 0;
};
}
