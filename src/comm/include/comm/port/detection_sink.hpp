#pragma once

#include <target/dto/detection_event.hpp>

namespace comm::port
{

class DetectionSink
{
public:
    virtual ~DetectionSink() = default;

    virtual void Write(const target::DetectionEvent& detection) = 0;
};

} // namespace comm::port
