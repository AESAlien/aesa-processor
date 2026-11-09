#pragma once

#include "domain/detection_result.hpp"

#include <target/dto/detection_event.hpp>

namespace target
{

class DetectionResultFactory
{
public:
    static DetectionResult from(const DetectionEvent& detection);
};

} // namespace target
