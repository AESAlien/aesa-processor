#pragma once

#include <target/dto/detection_event.hpp>

namespace target
{

class DetectionSource
{
public:
    virtual ~DetectionSource() = 0;

    virtual void read(const DetectionEvent& detection) = 0;
};

inline DetectionSource::~DetectionSource() = default;

} // namespace target
