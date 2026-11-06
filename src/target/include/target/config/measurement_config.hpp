#pragma once

#include <math/quantity/length.hpp>

#include <optional>

namespace target
{

class MeasurementConfig
{
public:
    class Builder
    {
    public:
        Builder& standardDeviation(math::Length value)
        {
            _standardDeviation = value;
            return *this;
        }

        MeasurementConfig build() const;

    private:
        std::optional<math::Length> _standardDeviation;
    };

    explicit MeasurementConfig(math::Length standardDeviation);

    const math::Length& standardDeviation() const noexcept;

private:
    math::Length _standardDeviation;
    void validate() const;
};

} // namespace target
