#pragma once

#include <math/quantity/linear_acceleration.hpp>

#include <optional>

namespace target
{

class TrackingConfig
{
public:
    class Builder
    {
    public:
        Builder& initialAccelerationStandardDeviation(math::LinearAcceleration value)
        {
            _initialAccelerationStandardDeviation = value;
            return *this;
        }

        Builder& processAccelerationStandardDeviation(math::LinearAcceleration value)
        {
            _processAccelerationStandardDeviation = value;
            return *this;
        }

        TrackingConfig build() const;

    private:
        std::optional<math::LinearAcceleration> _initialAccelerationStandardDeviation;
        std::optional<math::LinearAcceleration> _processAccelerationStandardDeviation;
    };

    TrackingConfig(
        math::LinearAcceleration initialAccelerationStandardDeviation,
        math::LinearAcceleration processAccelerationStandardDeviation
    );

    const math::LinearAcceleration& initialAccelerationStandardDeviation() const noexcept;
    const math::LinearAcceleration& processAccelerationStandardDeviation() const noexcept;

private:
    math::LinearAcceleration _initialAccelerationStandardDeviation;
    math::LinearAcceleration _processAccelerationStandardDeviation;
    void validate() const;
};

} // namespace target
