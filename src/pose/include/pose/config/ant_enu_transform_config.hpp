#pragma once

#include <math/linear_algebra/square_matrix.hpp>

#include <optional>

namespace pose
{

class AntEnuTransformConfig
{
public:
    class Builder
    {
    public:
        Builder& mountAntToBodyRotation(math::SquareMatrix value);

        AntEnuTransformConfig build() const;

    private:
        std::optional<math::SquareMatrix> _mountAntToBodyRotation;
    };

    explicit AntEnuTransformConfig(math::SquareMatrix mountAntToBodyRotation);

    const math::SquareMatrix& mountAntToBodyRotation() const noexcept;

private:
    math::SquareMatrix _mountAntToBodyRotation;
    void validate() const;
};

} // namespace pose
