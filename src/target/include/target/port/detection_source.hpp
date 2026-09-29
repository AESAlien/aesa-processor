#pragma once

#include <target/dto/detection_dto.hpp>

namespace target {

class DetectionSource
{
public:
    virtual ~DetectionSource() = 0;
    
    virtual void read(const DetectionDto& detection) = 0;
};

inline DetectionSource::~DetectionSource() = default;

} // namespace target
