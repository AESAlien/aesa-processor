#pragma once

#include <pose/domain/enu_geo_transform.hpp>

#include <utility>

namespace pose
{
struct SetEnuGeoTransformCommand
{
    explicit SetEnuGeoTransformCommand(EnuGeoTransform transform)
        : transform(std::move(transform))
    {
    }

    EnuGeoTransform transform;
};
}
