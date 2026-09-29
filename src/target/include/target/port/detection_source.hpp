#pragma once

#include <target/dto/detection.hpp>

namespace target {

class DetectionSource
{
public:
    virtual ~DetectionSource() = 0;
    
    virtual void read(const Detection& detection) = 0;
};

inline DetectionSource::~DetectionSource() = default;

} // namespace target
