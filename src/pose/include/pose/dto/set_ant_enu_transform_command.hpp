#pragma once

#include <pose/domain/ant_enu_transform.hpp>

#include <utility>

namespace pose
{
struct SetAntEnuTransformCommand
{
    explicit SetAntEnuTransformCommand(AntEnuTransform transform) : transform(std::move(transform))
    {
    }

    AntEnuTransform transform;
};
}
