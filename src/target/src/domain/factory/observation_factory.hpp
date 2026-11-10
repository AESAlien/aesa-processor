#pragma once

#include "domain/observation.hpp"

#include <target/dto/detection_event.hpp>

#include <vector>

namespace target
{

class ObservationFactory
{
public:
    static std::vector<Observation> from(const DetectionEvent& detection);
};

} // namespace target
