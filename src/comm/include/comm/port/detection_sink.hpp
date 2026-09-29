#pragma once

#include <target/dto/detection_dto.hpp>

namespace comm::port
{

class DetectionSink
{
public:
    virtual ~DetectionSink() = default;

    virtual void write(const target::DetectionDto& detection) = 0;
};

} // namespace comm::port
